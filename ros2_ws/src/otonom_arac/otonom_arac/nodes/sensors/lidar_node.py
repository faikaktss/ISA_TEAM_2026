
import math
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Int32

try:
    from rplidar import RPLidar
    RPLIDAR_AVAILABLE = True
except ImportError:
    RPLIDAR_AVAILABLE = False

try:
    from sklearn.cluster import DBSCAN
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False


class ClusterTracker:
    """Kümelere A/B/C... gibi kalıcı kimlikler atar (2024-2025 mantığı)."""

    HAREKET_ESIGI = 80   
    ESLESME_MESAFE = 300 

    def __init__(self):
        self._sonraki_id = 0
        self._kümeler = {}   

    def _yeni_harf(self):
        harf = chr(ord('A') + self._sonraki_id % 26)
        self._sonraki_id += 1
        return harf

    def guncelle(self, mevcutlar):
        """
        mevcutlar: [(cx, cy), ...] — bu frame'deki küme merkezleri
        döndürür: {id_harf: {'merkez': (x,y), 'hareketli': bool}}
        """
        eslesmemis_ids = set(self._kümeler.keys())
        yeni_kumeler = {}

        for cx, cy in mevcutlar:
            en_yakin_id = None
            en_yakin_d = float('inf')
            for kid, kdata in self._kümeler.items():
                px, py = kdata['merkez']
                d = math.hypot(cx - px, cy - py)
                if d < en_yakin_d and d < self.ESLESME_MESAFE:
                    en_yakin_d = d
                    en_yakin_id = kid

            if en_yakin_id is not None:
                eslesmemis_ids.discard(en_yakin_id)
                gecmis = self._kümeler[en_yakin_id]['gecmis']
                gecmis.append((cx, cy))
                if len(gecmis) > 10:
                    gecmis.pop(0)
                hareketli = self._hareket_var(gecmis)
                yeni_kumeler[en_yakin_id] = {'merkez': (cx, cy), 'gecmis': gecmis, 'hareketli': hareketli}
            else:
                harf = self._yeni_harf()
                yeni_kumeler[harf] = {'merkez': (cx, cy), 'gecmis': [(cx, cy)], 'hareketli': False}

        self._kümeler = yeni_kumeler
        return yeni_kumeler

    def _hareket_var(self, gecmis):
        if len(gecmis) < 3:
            return False
        ilk_x, ilk_y = gecmis[0]
        son_x, son_y = gecmis[-1]
        return math.hypot(son_x - ilk_x, son_y - ilk_y) > self.HAREKET_ESIGI


class EngelTakip:
    """
    İleri yay (60-120 derece) analizi ile engel durumu döndürür.
      0 = engel yok        (>500 mm)
      1 = durağan engel    (300-500 mm)
      2 = hareketli/kritik (<300 mm veya hareket eden)
    """

    KRITIK_MESAFE = 300   # mm
    UYARI_MESAFE  = 500   # mm
    YAY_MIN = 60
    YAY_MAX = 120

    def degerlendir(self, kümeler, tarama_noktalari):
        """
        kümeler: ClusterTracker.guncelle() çıktısı
        tarama_noktalari: [(aci_derece, mesafe_mm), ...]
        """
        on_noktalar = [
            d for a, d in tarama_noktalari
            if self.YAY_MIN <= a <= self.YAY_MAX and 0 < d < 6000
        ]
        if not on_noktalar:
            return 0

        min_mesafe = min(on_noktalar)

        for kdata in kümeler.values():
            if kdata['hareketli']:
                cx, cy = kdata['merkez']
                aci = math.degrees(math.atan2(cx, cy)) % 360
                if self.YAY_MIN <= aci <= self.YAY_MAX:
                    return 2

        if min_mesafe < self.KRITIK_MESAFE:
            return 2
        elif min_mesafe < self.UYARI_MESAFE:
            return 1
        return 0


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        self.declare_parameter('lidar_port', '/dev/ttyUSB1')
        self.declare_parameter('lidar_baudrate', 115200)

        lidar_port     = self.get_parameter('lidar_port').value
        lidar_baudrate = self.get_parameter('lidar_baudrate').value

        # Publisher'lar
        self.scan_publisher     = self.create_publisher(LaserScan,         '/lidar/scan',      10)
        self.distance_publisher = self.create_publisher(Float32MultiArray, '/lidar/distances', 10)
        self.obstacle_publisher = self.create_publisher(Int32,             '/lidar/obstacle',  10)

        # Algoritma bileşenleri
        self.tracker     = ClusterTracker()
        self.engel_takip = EngelTakip()
        self._tarama     = []   # [(aci, mesafe), ...]
        self._tarama_lock = threading.Lock()
        self._running = True

        # DBSCAN instance (her frame'de yeniden oluşturmak yerine)
        self._dbscan = DBSCAN(eps=200, min_samples=4) if SKLEARN_AVAILABLE else None

        # TERMINAL: takip değişkenleri
        self._last_engel = -1
        self._last_scan_time = 0.0
        self._lidar_scan_count = 0
        self._lidar_last_status = 0.0
        self._lidar_on_mesafe = 0.0
        self._lidar_nokta_sayisi = 0

        # RPLidar bağlantısı
        self.lidar = None
        if RPLIDAR_AVAILABLE:
            try:
                self.lidar = RPLidar(lidar_port, baudrate=lidar_baudrate)
                self.lidar.start_motor()
                self.get_logger().info(f'RPLidar bağlandı: {lidar_port}')
                # TERMINAL: başlangıç logu
                print(f'[LIDAR] Başlatıldı | topic: /lidar/scan subscribe edildi | Port: {lidar_port}', flush=True)
            except Exception as e:
                self.get_logger().warn(f'RPLidar bağlanamadı: {e}')
                # TERMINAL: bağlantı hatası
                print(f'[LIDAR] ✗ Bağlantı kurulamadı: {lidar_port} — {e}', flush=True)
                self.lidar = None
        else:
            self.get_logger().warn('rplidar kütüphanesi yok. pip install rplidar-roboticia')
            print('[LIDAR] ⚠ rplidar kütüphanesi yok — donanım bağlanamaz', flush=True)

        if not SKLEARN_AVAILABLE:
            self.get_logger().warn('sklearn yok — DBSCAN devre dışı. pip install scikit-learn')

        # Lidar okuma ayrı thread'de — iter_measures() bloklayıcı!
        if self.lidar is not None:
            self._read_thread = threading.Thread(target=self._lidar_read_loop, daemon=True)
            self._read_thread.start()

        # 10 Hz publish timer — sadece son taraması yayınlar
        self.timer = self.create_timer(0.1, self.timer_callback)
        # TERMINAL: 1s özet timer
        self._status_timer = self.create_timer(10.0, self._terminal_status_1s)

    def _lidar_read_loop(self):
        """Arka plan thread'i: iter_measures() burada bloklar, ana executor etkilenmez."""
        try:
            for yeni_scan, kalite, aci, mesafe in self.lidar.iter_measures(max_buf_meas=500):
                if not self._running:
                    break
                if kalite > 0 and mesafe > 0:
                    with self._tarama_lock:
                        if yeni_scan and self._tarama:
                            # Yeni tarama başladı — mevcut taramayı kaydet
                            pass  # timer_callback'te publish edilecek
                        if yeni_scan:
                            self._tarama = []
                        self._tarama.append((float(aci), float(mesafe)))
        except Exception as e:
            self.get_logger().error(f'Lidar okuma hatası: {e}')
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
            except Exception:
                pass

    def timer_callback(self):
        """10 Hz: Son taramayı al ve yayınla — ASLA bloklamaz."""
        with self._tarama_lock:
            tarama = list(self._tarama) if self._tarama else None

        if not tarama:
            return

        try:
            self._yayinla(tarama)
        except Exception as e:
            self.get_logger().error(f'Lidar yayın hatası: {str(e)}')

    def _yayinla(self, tarama):
        stamp = self.get_clock().now().to_msg()

        scan_msg = LaserScan()
        scan_msg.header.stamp    = stamp
        scan_msg.header.frame_id = 'lidar_link'
        scan_msg.angle_min       = 0.0
        scan_msg.angle_max       = 2 * math.pi
        scan_msg.angle_increment = math.radians(1.0)
        scan_msg.time_increment  = 0.0
        scan_msg.range_min       = 0.05
        scan_msg.range_max       = 6.0

        ranges = [float('inf')] * 360
        for aci, mesafe in tarama:
            idx = int(aci) % 360
            m   = mesafe / 1000.0  # mm → m
            if m < ranges[idx]:
                ranges[idx] = m
        scan_msg.ranges = ranges
        self.scan_publisher.publish(scan_msg)

        def min_yay(a1, a2):
            vals = [d for a, d in tarama if a1 <= a % 360 <= a2 and 0 < d < 6000]
            return min(vals) if vals else 0.0

        on   = min_yay(60,  120)
        sol  = min_yay(150, 210)
        arka = min_yay(240, 300)
        sag_vals = [d for a, d in tarama if ((330 <= a % 360 <= 360) or (0 <= a % 360 <= 30)) and 0 < d < 6000]
        sag  = min(sag_vals) if sag_vals else 0.0

        dist_msg = Float32MultiArray()
        dist_msg.data = [float(on), float(sol), float(sag), float(arka)]
        self.distance_publisher.publish(dist_msg)

        engel_durum = 0
        if self._dbscan is not None and tarama:
            # Polar → Kartezyen (mm)
            pts = []
            for aci, mesafe in tarama:
                if 0 < mesafe < 6000:
                    rad = math.radians(aci)
                    pts.append([mesafe * math.cos(rad), mesafe * math.sin(rad)])

            if len(pts) >= 4:
                pts_arr = np.array(pts)
                labels  = self._dbscan.fit_predict(pts_arr)
                # Küme merkezleri
                merkezler = []
                for lbl in set(labels):
                    if lbl == -1:
                        continue
                    mask = labels == lbl
                    merkezler.append(tuple(pts_arr[mask].mean(axis=0)))

                kümeler = self.tracker.guncelle(merkezler)
                engel_durum = self.engel_takip.degerlendir(kümeler, tarama)

        obs_msg = Int32()
        obs_msg.data = engel_durum
        self.obstacle_publisher.publish(obs_msg)

        # TERMINAL: engel durumu değişince aninda bas
        if engel_durum != self._last_engel:
            if engel_durum == 0:
                print('[LIDAR] ✓ Engel kalktı', flush=True)
            elif engel_durum == 1:
                _m = on / 1000.0 if on > 0 else 0.0
                print(f'[LIDAR] ⚠ ENGEL TESPİT: durağan | mesafe={_m:.2f}m', flush=True)
            elif engel_durum == 2:
                _m = on / 1000.0 if on > 0 else 0.0
                print(f'[LIDAR] ⚠ ENGEL TESPİT: hareketli/kritik | mesafe={_m:.2f}m', flush=True)
            self._last_engel = engel_durum
        # TERMINAL: özet için takip
        import time as _time
        self._last_scan_time = _time.time()
        self._lidar_on_mesafe = on / 1000.0 if on > 0 else 0.0
        self._lidar_nokta_sayisi = len(tarama)

    # TERMINAL: 1 saniyede bir lidar özeti
    def _terminal_status_1s(self):
        import time as _time
        _now = _time.time()
        if self._last_scan_time > 0 and (_now - self._last_scan_time) > 3.0:
            print(f'[LIDAR] ✗ Tarama verisi gelmiyor! ({_now-self._last_scan_time:.0f}s süredir)', flush=True)
        elif self._last_scan_time > 0:
            print(
                f'[LIDAR] ✓ Aktif | ön_mesafe={self._lidar_on_mesafe:.2f}m | engel={self._last_engel}',
                flush=True)

    def destroy_node(self):
        self._running = False
        if self.lidar is not None:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
