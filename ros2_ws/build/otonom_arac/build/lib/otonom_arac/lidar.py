import math
import numpy as np
import threading
from PyQt5.QtCore import QTimer
from adafruit_rplidar import RPLidar
from sklearn.cluster import DBSCAN
import pyqtgraph as pg
from PyQt5.QtWidgets import QMainWindow
import time

#Todo: Genel yapısı şöyledir: Otonom araç için lidar tabanlı engel tespit sistemidir }


class ClusterTracker:
    #Todo:Araba hareket ederken hala aynı arabamı olduğunu anlamasını sağlayana kod yapısı 
    #Todo: threshold = 500mm olarak belirlenmiş bu mesafeyi aşmıyor ise araba aynı arabadır
    def __init__(self, distance_threshold=500):
        self.label_to_name = {}
        self.name_to_centroid = {}#Todo: Merkez koordinataları
        self.next_name_index = 0
        self.distance_threshold = distance_threshold#Todo:eşik sayaç

    #Todo:Arabalara isim verir
    def _get_next_name(self):
        return chr(ord('A') + self.next_name_index)

    def update_clusters(self, points, labels):
        #Todo: Grup numarası herekese özel
        unique_labels = set(labels)
        current_centroids = {}
        #Todo: Merkez noktalarını bulur
        for label in unique_labels:
            #Todo: -1 gürültü demektir bu nesneler atlanır
            if label == -1:
                continue
            #Todo: Nesnenin tam orta noktası bulunur
            cluster_points = points[labels == label]
            centroid = np.mean(cluster_points, axis=0)#Todo: Ortalama merkez alınır
            current_centroids[label] = centroid

        new_label_to_name = {}
        used_names = set()

        for label, centroid in current_centroids.items():
            matched = False
            #Todo: Eski nesneleri tek tek gezeriz
            for name, prev_centroid in self.name_to_centroid.items():
                #Todo: Yeni ve eski nesneler arasındaki mesafe hesaplanır
                dist = np.linalg.norm(centroid - prev_centroid)
                #Todo: Eğer yer değiştirmişlerse yeni yeri güncellenir
                if dist < self.distance_threshold and name not in used_names:
                    new_label_to_name[label] = name
                    self.name_to_centroid[name] = centroid
                    used_names.add(name)
                    matched = True
                    break
                #Todo: Hiç biri eski nesneye yakın değilse odaya yeni bir nesne girdiği anlaşılır
            if not matched:
                new_name = self._get_next_name()
                self.label_to_name[label] = new_name
                self.name_to_centroid[new_name] = centroid
                new_label_to_name[label] = new_name
                used_names.add(new_name)
                self.next_name_index += 1

        return new_label_to_name


class EngelTakip:
    #Todo: Nesnelerin hareket edip etmediğini takip eden bir sınıf
    #Todo: Fark eşiğide titreşimleri anlamak için kullanılır
    def __init__(self, x_fark_eşiği=50, history_length=3, uzun_menzil=5000, orta_menzil=3000, kısa_menzil=3000):
        self.x_gecmisleri = {}
        self.x_fark_eşiği = x_fark_eşiği
        self.history_length = history_length
        self.uzun_menzil = uzun_menzil
        self.orta_menzil = orta_menzil
        self.kısa_menzil = kısa_menzil


    #Todo: Nesnelerin hareket edip etmediğini kontrol eder
    def guncelle_ve_kontrol_et(self, clusters):
        sonuc = {}

        for name, (x, y) in clusters.items():
            #Todo: Nesnenin uzaklığı ve açısı hesaplanır
            uzaklık = math.sqrt(x ** 2 + y ** 2)
            #Todo: Açı 0-360 derece arasında normalize edilir
            açı = math.degrees(math.atan2(y, x)) % 360

            if (60 < açı < 120):
                #Todo: Bu nesne hiç takip edilmemişse  ona ait boş bir geçmiş başlatılır
                if name not in self.x_gecmisleri:
                    self.x_gecmisleri[name] = []
                self.x_gecmisleri[name].append(x)
                #Todo: Geçmiş uzunluğu sınırlandırılır
                #Todo: Bu bellek tasarrufu sağlar
                if len(self.x_gecmisleri[name]) > self.history_length:
                    self.x_gecmisleri[name] = self.x_gecmisleri[name][-self.history_length:]
                #Todo: Karşılaştırma yapmajk için yeterli geçmiş veri toplandı mı
                if uzaklık < self.uzun_menzil and len(self.x_gecmisleri[name]) >= self.history_length:
                    #Todo: ilk veriyi alır 
                    eski_x = self.x_gecmisleri[name][0]
                    #Todo: Şimdiki veri ile karşılaştırır
                    fark = abs(x - eski_x)
                    #Todo: Fark eşiğinden büyükse nesne hareket ediyor demektir
                    hareketli = fark > self.x_fark_eşiği
                    sonuc[name] = ("hareketli" if hareketli else "hareketsiz")
        return sonuc
#Todo: Lidar verilerini görselleştirmek için özel bir PyQtGraph widget'ı
class LidarPlotWidget(pg.PlotWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        #Todo: x ve y eksen oranlarını sabitler
        self.setAspectLocked(True)


        #Todo:Nokta bulutu çizmek için özel bir nesne oluşturulur
        self.scatter = pg.ScatterPlotItem(size=5, pen=None)
        self.addItem(self.scatter)
        #Todo: Görüntülenecek alan sınırları belirlenir
        self.setXRange(-5000, 5000)
        self.setYRange(-5000, 5000)
        #Todo:Klavuz çizgileri için bir liste
        self.guide_items = []

    def update_points(self, points, labels, label_to_name):
        self._draw_guides()
        #Todo: Noktaları renklendir ve çiz
        spots = []
        color_map = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255),
            (255, 255, 0), (255, 0, 255), (0, 255, 255)
        ]
        #Todo: Her nokta için uygun renk belirlenir
        for idx, (x, y) in enumerate(points):
            label = labels[idx]
            #Todo: Gürültü noktaları gri renkle gösterilir
            if label == -1:
                color = (100, 100, 100)
            else:
                cname = label_to_name.get(label, '?')
                color = color_map[ord(cname) % len(color_map)]
                #Todo: Noktalar çizilir
            spots.append({'pos': (x, y), 'brush': pg.mkBrush(*color)})
            #Todo: Noktalar çizilir
        self.scatter.setData(spots)

    def _draw_guides(self):
        #Todo: Önceki kılavuz çizgilerini kaldır
        for item in self.guide_items:
            self.removeItem(item)
        self.guide_items.clear()

        ranges = [5000, 3000, 3000]
        colors = ['g', 'y', 'r']


        #Todo:yatay beyaz çizgi x ekseni boyunca
        white_line = pg.PlotCurveItem([-5000, 5000], [0, 0], pen=pg.mkPen('w', width=0.7))
        #Todo: Dikey beyaz çizgi y ekseni boyunca 
        white_liney = pg.PlotCurveItem([0, 0], [-20, 20], pen=pg.mkPen('w', width=0.7))
        #Todo: çizgileri grafiğe ekle ve silebilmek için kaydet
        self.addItem(white_line)
        self.addItem(white_liney)
        self.guide_items.append(white_line)
        self.guide_items.append(white_liney)

        #Todo: Her mesafe için daire çiz ve grafiğe ekle
        for r, color in zip(ranges, colors):
            theta = np.linspace(0, 2 * np.pi, 200)
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            curve = pg.PlotCurveItem(x, y, pen=pg.mkPen(color, width=0.7))
            self.addItem(curve)
            self.guide_items.append(curve)


class LidarSystem:
    #Todo: port = lidarın bağlı olduğu usb portu
    def __init__(self, port="COM7", enable_gui=False, print_clusters=False):
        self.port = port
        self.enable_gui = enable_gui
        self.print_clusters = print_clusters
        self.engel = 0
        self.durak_engel = 0

        self.uzun_menzil = 5000
        self.orta_menzil = 3000
        self.kısa_menzil = 3000


        #Todo: RpLidar bağlanıtısı kuruluyor
        try:
            self.lidar = RPLidar(None, self.port, timeout=3)
        except Exception as e:
            print(f"Error initializing RPLidar: {e}")
            self.lidar = None
        self.lidar.clear_input()
        self.running = False

        #Todo: Görsel harita
        self.plot_widget = LidarPlotWidget() #if enable_gui else None

        #Todo: Kümeleme ve takip için nesneler oluşturulur
        self.cluster_tracker = ClusterTracker()
        self.engel_takip = EngelTakip(
            uzun_menzil=self.uzun_menzil,
            orta_menzil=self.orta_menzil,
            kısa_menzil=self.kısa_menzil
        )

        #Todo: En son tarama verilerini saklamak için değişkenler
        self.latest_points = None
        self.latest_labels = None
        self.label_name_map = {}


        #Todo: GUI ayarları
        #Todo: Her 100ms de bir grafik güncellenir
        if self.enable_gui:
             self.window = QMainWindow()
             self.window.setCentralWidget(self.plot_widget)
             self.window.setWindowTitle("LIDAR Viewer")
             self.timer = QTimer()
             self.timer.timeout.connect(self._update_plot)
             self.timer.start(100)

    def _update_plot(self):
        if self.plot_widget and self.latest_points is not None and self.latest_labels is not None:
            self.plot_widget.update_points(self.latest_points, self.latest_labels, self.label_name_map)


    def start(self):
        self.running = True
        threading.Thread(target=self._scan_loop, daemon=True).start()

    def _scan_loop(self):

        #Todo: Tarama döngüsü başlatır
        for scan in self.lidar.iter_scans():
            if not self.running:
                break
            points = []
            #Todo: Her tarama noktası için açı ve mesafe bilgisi geliyor 
            #Todo: Kutupsal koordinatları Kartezyen koordinatlara dönüştürülüyor
            for (_, angle, distance) in scan:
                if angle >= 360:
                    continue
                #Todo: Kutupsal koordinatları Kartezyen koordinatlara dönüştür
                x = distance * math.sin(math.radians(angle))
                y = distance * math.cos(math.radians(angle))
                points.append((x, y))


            #todo: Burada kümeleme yapılır yani yakın noktalara sahip nesneler gruplanır
            if points:
                points_np = np.array(points)
                #Todo: DBSCAN kümeleme algoritması uygulanır birbirine yakın noktaları gruplar
                clustering = DBSCAN(eps=200, min_samples=4).fit(points_np)
                labels = clustering.labels_
                label_to_name = self.cluster_tracker.update_clusters(points_np, labels)

                self.latest_points = points_np
                self.latest_labels = labels
                self.label_name_map = label_to_name

                #Todo: Kümelenmiş kısımaların bilgilerini al
                clusters = self.get_clusters()
                hareket_durumu = self.engel_takip.guncelle_ve_kontrol_et(clusters)
                durak_mezilinde_engel_var = False
                orta_menzilde_engel_var = False
                #Todo: Her küme için merkez noktasını al 
                #Todo: uzaklık hesapla ve engel kontrolü yap
                for name, center in clusters.items():
                    x, y = center
                    uzaklık = math.sqrt(x**2 + y**2)
                    açı = math.degrees(math.atan2(y, x)) % 360
                    hareket = hareket_durumu.get(name, "")

                    #Todo: Engel robotun önündemi 
                    if (60 < açı < 120):
                        if self.kısa_menzil < uzaklık < self.orta_menzil:
                            print(f"Küme {name}: {center} uzun menzilde {hareket}")
                        elif uzaklık < self.orta_menzil:
                            print(f"Küme {name}: {center} {hareket} engel")
                            orta_menzilde_engel_var = True
                            if hareket == "hareketli":
                                self.engel = 2
                                
                            else:
                                self.engel = 1
                    #if(0<açı<90):
                    #    if  uzaklık < 2000:
                     #       durak_mezilinde_engel_var = True
                      #      self.durak_engel = 1
                    for (_, angle, distance) in scan:
                        if angle >= 360:
                            continue
                        #Todo: sağ tarafta 3 metreden yakın bir şey var mıdır
                        if 0< angle < 90 and distance<3000:
                            durak_mezilinde_engel_var = True
                            self.durak_engel=1

                if not durak_mezilinde_engel_var:
                    self.durak_engel = 0

                if not orta_menzilde_engel_var:
                    self.engel = 0
                    print("engelll yooooooook")

    def stop(self):
        self.running = False
        self.lidar.stop()
        self.lidar.disconnect()

    #Todo: Kümelerin merkez noktalarını bul 
    def get_clusters(self):
        clusters = {}
        if self.latest_labels is None or self.latest_points is None:
            return clusters
        #Todo: Tekrarlanan etiketleri temizler
        labels = set(self.latest_labels)
        for label in labels:
            if label == -1:
                continue
            #Todo: Küme adı ve merkez noktasını al
            cname = self.label_name_map.get(label, '?')
            cluster_points = self.latest_points[self.latest_labels == label]
            centroid = np.mean(cluster_points, axis=0)
            clusters[cname] = centroid
        return clusters

    def get_current_data(self):
        return self.latest_points, self.latest_labels, self.label_name_map
