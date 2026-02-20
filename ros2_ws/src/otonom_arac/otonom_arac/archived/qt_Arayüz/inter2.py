import sys
import cv2
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout, QTextEdit
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QTextCursor, QIcon
from PyQt5.QtCore import QPointF, pyqtSignal, QObject
from qt_Arayüz.qt2 import ClickableLabel, Ui_Form
from qt_Arayüz.info_window import InfoWindow
from qt_Arayüz.threshold_settings import ThresholdSettingsWindow
from info import Info
from lidar import LidarSystem


#Todo: Otonom aracın ana kontrol panelidir
class ClickableLabel(QLabel):
    def __init__(self, index, parent=None):
        super().__init__(parent)
        #Todo: Label'ın kimlik numarasıdır
        self.index = index

    #Todo: Tıklama olayını yakalar
    def mousePressEvent(self, event):
        if self.parent() and hasattr(self.parent(), 'show_popup'):
            self.parent().show_popup(self.index)

    #Todo:Lidar noktalarını güncellemek için kullanılır
    def update_lidar(self,lidar_data):
        #Todo:boş bir tuval oluşturur
        pixmap = QPixmap(self.width(),self.height())
        #Todo:arka planı siyah yapar
        pixmap.fill(Qt.black)
        #Todo:Çizim işlemleri için QPainter kullanır
        painter = QPainter(pixmap)
        #Todo:Kenar yumuşatma etkinleştirir
        painter.setRenderHint(QPainter.Antialiasing)


        #Todo: Merkez noktayı belirler burası lidar'ın konumu olacak
        center = QPointF(self.width() / 2, self.height() / 2)
        scale = min(self.width(), self.height()) / 4000

        #Todo: Her bir lidar noktasını çizer
        for point in lidar_data:
            x, y, color = point
            #Todo: Koordinat dönüşümü yapar
            px = center.x() + x * scale
            py = center.y() - y * scale
            painter.setPen(QPen(QColor(*color), 4))
            painter.drawPoint(QPointF(px, py))

        painter.end()
        self.setPixmap(pixmap)


#Todo:Küçük bir görüntüye tıklandığında büyük bir penecerede gösterir
class ImagePopup(QWidget):
    def __init__(self, frame, title="Büyük Görüntü"):
        super().__init__()
        self.setWindowTitle(title)
        #Todo:Dikey yerleşim yerlerini oluşturur
        layout = QVBoxLayout()
        label = QLabel()
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        #Todo: Qt formatına çevir
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        #Todo: boyutlandır ve yerleştir
        label.setPixmap(pixmap.scaled(800, 600, Qt.KeepAspectRatio))
        layout.addWidget(label)
        self.setLayout(layout)
        self.resize(820, 620)
        
#Todo: Sonuç olarak iki yerde birden çıktı gözükür
class EmittingStream(QObject):
    #Todo: Birisi yazınca -> sinyal yapılır ve dinleyenler alır
    text_written = pyqtSignal(str)

    def __init__(self, original_stream, parent=None):
        super().__init__(parent)
        self.original_stream = original_stream  # Orijinal sys.stdout/sys.stderr

    def write(self, text):
        self.text_written.emit(str(text))      # PyQt sinyali yay
        self.original_stream.write(text)       # Gerçek terminale de yaz
        self.original_stream.flush()           # Hemen göster
    #Todo:flush çağrıldığında orijinal streami de flushlar
    def flush(self):
        self.original_stream.flush()


#Todo:Büyük bir pencereye log kısımlarını eklemeye yarar
class ConsoleWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Console Output")
        self.resize(600, 400)
        self.setStyleSheet("background-color: rgb(94, 94, 87);")
        self.setWindowIcon(QIcon(r"C:\Users\byznr\Desktop\17.07.2025\logo.jpg"))

        layout = QVBoxLayout()
        self.text_area = QTextEdit()
        self.text_area.setReadOnly(True)
        layout.addWidget(self.text_area)
        self.setLayout(layout)

    def append_text(self, text):
        self.text_area.moveCursor(QTextCursor.End)
        self.text_area.insertPlainText(text)
        self.text_area.moveCursor(QTextCursor.End)

# Todo: Aracın ekranı gibi düşünün
# Todo: Farklı kameraların görüntülerini ve lidar verilerini gösterir
class VideoApp(QWidget):
    # Todo: Uygulama kapandığında sinyal gönderir
    exit_signal = pyqtSignal()

    def __init__(self,info):
        # Todo: QWidget başlatılır
        super().__init__()
        self.ui = Ui_Form() # Todo: Arayüz sınıfı başlatılır
        self.ui.setupUi(self)

        self.info = info #Todo: Sensör + algılama bilgilerini tutan nesne
        self.info_window =None 
        self.threshold_settings_window = None

        # Konsol penceresini oluştur
        self.console_window = ConsoleWindow() 

        # Orijinal stdout/stderr'i sakla
        self.original_stdout = sys.stdout
        self.original_stderr = sys.stderr

        # Yeni EmittingStream nesneleri oluştur, orijinalleri de aktar
        sys.stdout = EmittingStream(self.original_stdout)
        sys.stdout.text_written.connect(self.console_window.append_text)
        
        # Todo: Hatalar da GUI'de görünür
        sys.stderr = EmittingStream(self.original_stderr)
        sys.stderr.text_written.connect(self.console_window.append_text)


        self.ui.pushButton_console.clicked.connect(self.console_window.show)
        self.ui.pushButton_info.clicked.connect(self.show_info_window)
        self.ui.pushButton_Tbar.clicked.connect(self.show_threshold_settings)

        # Clickable QLabel'ları tanımlıyoruz
        self.ui.label_original = ClickableLabel(0, self)
        #self.ui.label_lidar = ClickableLabel(1, self)
        self.ui.label_tSignage = ClickableLabel(2, self)
        self.ui.label_birdeye = ClickableLabel(3, self)
        

        # Todo: Gösterilen framleri saklar
        self.current_frames = [None] * 4
        self.cap = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.process_frame)  # Video işleme fonksiyonuna bağladık
        self.ui.pushButton_exit.clicked.connect(self.exit_app)
        self.popup_window = None


    def exit_app(self):
        if hasattr(self):
            try:
                LidarSystem.stop()
                
            except Exception as e:
                print(f"Lidar kapanırken hata oluştu: {e}")
        self.exit_signal.emit()
        self.close()


    #Todo: Pencereyi direk değil butona basıldığında açılır
    def show_info_window(self):
        if self.info_window is None:
            self.info_window = InfoWindow(self.info)
        self.info_window.show()

    def show_threshold_settings(self):
        if self.threshold_settings_window is None:
            self.threshold_settings_window = ThresholdSettingsWindow(self)
        self.threshold_settings_window.show()
        self.threshold_settings_window.raise_()
        self.threshold_settings_window.activateWindow()

    def start_video(self):
        self.timer.start(30)
    #Todo: Görüntüleri ekrana basar
    def process_frame(self,station_frame,result,edges,bird,angle,lidar_data, signage_img):
        self.current_frames[0]=station_frame
        self.current_frames[1]=edges
        self.current_frames[2]=bird
        self.current_frames[3]=signage_img

        self.show_frame(self.ui.label_original, station_frame)
        #self.show_frame(self.ui.label_result, result)
        #self.show_frame(self.ui.label_edges, edges)
        self.show_frame(self.ui.label_birdeye, bird)

        if signage_img is not None:
            self.show_frame(self.ui.label_tSignage,signage_img)

        # if lidar_data is not None:
        #     self.ui.label_lidar.update_lidar(lidar_data)
    #Todo: İki kısım arasındaki renkleri düzeltir
    def show_frame(self, label, frame):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        label.setPixmap(pixmap.scaled(label.width(), label.height(), Qt.KeepAspectRatio))

    # Todo: Büyütülmüş bir pencerede göstermek için kullanılır
    def show_popup(self, index):
        if self.current_frames[index] is not None:
            self.popup_window = ImagePopup(self.current_frames[index])
            self.popup_window.show()

    # Todo : responsive bir görüntü elde etmek için kullanılır
    def resizeEvent(self, event):
        width = self.width()
        height = self.height()

        self.ui.label_original.setGeometry(int(0.01 * width), int(0.01 * height), int(0.45 * width), int(0.45 * height))
        self.ui.label_title_original.setGeometry(int(0.01 * width), int(0.01 * height), int(0.45 * width), int(0.05 * height))  # Başlık yeri
        self.ui.label_title_original.setStyleSheet("color: white; font-size: 20px; font-weight: bold;")

        self.ui.label_lidar.setGeometry(int(0.5 * width), int(0.01 * height), int(0.45 * width), int(0.45 * height))
        self.ui.label_title_lidar.setGeometry(int(0.5 * width), int(0.01 * height), int(0.45 * width), int(0.05 * height))  # Başlık yeri
        self.ui.label_title_lidar.setStyleSheet("color: white; font-size: 20px; font-weight: bold;")
        
        self.ui.label_tSignage.setGeometry(int(0.01 * width), int(0.5 * height), int(0.45 * width), int(0.4 * height))
        self.ui.label_title_tSignage.setGeometry(int(0.01 * width), int(0.5 * height), int(0.45 * width), int(0.05 * height))  # Başlık yeri
        self.ui.label_title_tSignage.setStyleSheet("color: white; font-size: 20px; font-weight: bold;")
        
        self.ui.label_birdeye.setGeometry(int(0.5 * width), int(0.5 * height), int(0.45 * width), int(0.4 * height))
        self.ui.label_title_birdeye.setGeometry(int(0.5 * width), int(0.5 * height), int(0.45 * width), int(0.05 * height))  # Başlık yeri
        self.ui.label_title_birdeye.setStyleSheet("color: white; font-size: 20px; font-weight: bold;")
        
        self.ui.pushButton_exit.setGeometry(int(0.4 * width), int(0.92 * height), int(0.18 * width), int(0.07 * height))
        self.ui.pushButton_info.setGeometry(int(0.6 * width), int(0.92 * height), int(0.10 * width), int(0.07 * height))
        self.ui.pushButton_Tbar.setGeometry(int(0.3 * width), int(0.92 * height), int(0.09 * width), int(0.07 * height))



        super().resizeEvent(event)

