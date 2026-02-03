# Todo: Kamera + Lidar + Yapay Zekâ verilerini işleyip, karar vererek, donanımı kontrol eden otonom bir sistem oluşturur.
from LaneDetect import LaneDetection # Todo: Şerit tespiti için gerekli sınıf
from camera import cam#Todo: Kamera ile görüntü almak için
from PyQt5.QtWidgets import QApplication#Todo: GUI uygulaması için
from lidar import LidarSystem # Todo: Lidar sensöründen mesafe verisi almak için
from qt_Arayüz.inter2 import VideoApp # Todo: GUI arayüzü için
from ultralytics import YOLO # Todo: YOLO modelini kullanmak için
from utils import turnInfo, info_to_geojson_feature # Todo: Yardımcı fonksiyonlar 
from info import Info#Todo: Bilgi yönetimi için
from arduino import Teensy, Arduino # Todo: Donanım kısmıyla iletişime geçmek için
#from karar import Command
from lidar import LidarSystem#Todo: Lidar sensöründen mesafe verisi almak için
import sys#Todo: Sistem işlemleri için
import cv2#Todo: Görüntü işleme için
import threading # Todo: Çoklu iş parçacığı yönetimi için
from sonKararTabela import komutcu2, komutcu # Todo: Tabela ve komut işlemleri için
import time #Todo: Zamanlama işlemleri için
from realsense import RealSenseCam#Todo: RealSense kamera ile nesne tespiti için

class_dict = {
    0: "kirmizi",
    1: "yesil",
    2: "park",
    3: "park_engelli",
    4: "park_yasak",
    5: "sag",
    6: "sol",
    7: "sag_yasak",
    8: "sol_yasak",
    9: "dur",
    10: "durak",
    11: "girilmez",
    12: "ileri_sag",
    13: "ileri_sol",
    14: "kavsak",
    15: "sagdan_devam",
    16: "soldan_devam",
    17: "park_yasakiki",
    18: "yaya",
    19: "tunel",
    20: "ileri",
    21: "cift_yon"
}

def deneme():
    for i in range(99999):
        print(i)
#Todo: Teensy ve Arduino donanım arayüzlerini başlatır
teensy = Teensy()

info = Info()#Todo: Bilgi yönetimi için nesne oluşturur
arduino = Arduino()#Todo: Arduino donanım arayüzü için nesne oluşturur
komutcu1 = komutcu(info, teensy, arduino)#Todo: Komut işlemleri için nesne oluşturur
komutcu2o = komutcu2(info, teensy, arduino)#To
# Todo: Aynı anda çalışan iki iş parçacığı oluşturur
th1 = threading.Thread(target=komutcu1.komut)
th2 = threading.Thread(target=komutcu2o.komut)

def main():
    #Todo: Robotun grafik arayüzünü başlatır
    app = QApplication(sys.argv)  
    window = VideoApp(info) #Todo: GUI penceresi oluşturur
    model = YOLO("modeller/best.pt")
    lane_detection = LaneDetection() # Todo: Şerit tespiti için nesne oluşturur
    zed = cam(info=info)  # ZED kamerada model_path kaldırıldı, sadece şerit takibi için # Todo: Kamera nesnesi
    realsense = RealSenseCam(model_path="modeller/best.pt")  # RealSense kamera nesnesi, nesne tespiti için # Todo: RealSense kamera nesnesi, nesne tespiti için
    # elma = LidarSystem(port="COM7", enable_gui=True, print_clusters=True)
    #komut = Command(info, teensy, arduino, zed)

    # lidar_widget = elma.plot_widget
    # lidar_widget.setParent(window.ui.label_lidar)
    # lidar_widget.setGeometry(0, 0, window.ui.label_lidar.width(), window.ui.label_lidar.height())
    # lidar_widget.show()
    # elma.start()

    #cap = cv2.VideoCapture("duzyol.mp4")
    window.show() 
    tabela = []

    while window.isVisible():
        frame, point_cloud = zed.img_and_point_cloud()
        if frame is None or point_cloud is None:
            break

        # RealSense’den nesne tespiti için frame ve nokta bulutu al
        rs_object_detection=realsense.detect_objects()
        # Şerit ve açı işlemleri
        station_result, angle = lane_detection.station_bird_eye_view(frame)
        result, edges = lane_detection.process(frame, 1920, 1080)
        bird, bird_canny, angle = lane_detection.bird_eye_view(frame)

        # RealSense ile nesne tespiti
        #signage_img = realsense.object_detection(rs_frame, rs_point_cloud)

        # Görseli GUI'de göster
        window.process_frame(station_result, result, edges, bird, angle, None, signage_img=rs_object_detection)

        

        # original ve bird eye frameleri infoya aktarır
        info.set_originalFrame(frame)
        info.set_birdEyeFrame(bird)
        info.set_edgeBirdEye(bird_canny)
        cv2.imshow("bu kbnin edgelisi", info.get_edgeBirdEye())
        #cv2.imshow("RGB FRAME",rs_frame)


        # lane detection sonucunu alalım
        lane = lane_detection.lane_position
        info.set_lane(lane)

        #kayma 
        """""""""""""""""""""""""""
        if(lane_detection.kaymaEsigi_ve_VirajKontrol()):
             print("midpoint referans alindi")
             info.set_angle(lane_detection.midPointVectorAngle)  
             burada kayma buyukse ve tek serit varsa kontrol ediliyor
             bunun ayarı icin lane detection class git self.kaymaesik kaymanın ne kadar olabilecegini ayarla bu ayardan fazla ise kayma, midpoint kullan
             aynı zamanda midpoint yoksa iki serit bulunamamistir ve arac virajdadır cunku tek cizgi vardır bu yüzden false doner
             asagidaki kusbakisa ointe gider 
        else:
        """""""""""""""""""""""""""""

        print("kusbakis referans alindi")
        #info.set_angle(angle)  
        
        # Bilgileri güncelle
        if angle is not None:
            if angle < 0:
                angle -= 4
            elif angle > 0:
                angle += 3

        info.set_angle(angle)
        #arduino.setValue(info.get_angle())
        # info.set_engel(elma.engel)
        # info.set_durak_engel(elma.durak_engel)

        #komut.engel_kontrol()
        #info.set_encoder(arduino.encoder_distance())
        #info.set_imu(teensy.get_imu())
        #info.set_gps(gps_module.get_position())
        #info.set_tabela(class_dict.get(zed.name))
                            

        # Eğer info penceresi açıksa GUI'yi güncelle
        if window.info_window is not None:
            window.info_window.update_info()

        sol_red_count, sag_red_count = turnInfo(frame, 3, 3)

        info.set_solRedCount(sol_red_count)
        info.set_sagRedCount(sag_red_count)

        # Tespit edilen tabelayı kaydet (RealSense’den alınan isimle)
        tabela_adi = class_dict.get(realsense.name)
        tabela.append(tabela_adi)

        # En fazla 10 taneyi tut
        if len(tabela) > 10:
            tabela.pop(0)

        #  Mesafeyi konsola yazdır
        if tabela_adi and realsense.distance is not None:
            print(f"Tespit: {tabela_adi} | Mesafe: {realsense.distance:.2f} cm")

        # Karar fonksiyonuna mesafeyi de gönder
        #komut.karar(tabela, sol_red_count, sag_red_count)
        # info_to_geojson_feature(info)
        info.set_tabela(tabela_adi)
        
        kb = frame.copy()
        gray = cv2.cvtColor(kb, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (25, 25), 0)
        _, kb2 = cv2.threshold(blur, 170, 255, cv2.THRESH_BINARY)
        kb2 = cv2.Canny(kb2, 50, 150)

        #print(info.get_tabela())
        if kb is not None:
            durak_yakalayici = kb2[850:1080, 1400:1920]
            cv2.imshow("durak", durak_yakalayici)
            info.set_durakPixel(cv2.countNonZero(durak_yakalayici))
            print(info.get_durakPixel())

        if cv2.waitKey(1) & 0xFF == ord('q'):
            # elma.stop()
            break

    sys.exit(app.exec_())
    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    #th1.start()  # tabelalı da bunu aç diğerini yorum satır yap
    th2.start()  # tabelasızda bunu aç diğerini yorum satır yap
    main()
    