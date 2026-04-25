import os
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Float32MultiArray
import numpy as np

import cv2

_IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# Point cloud da BEST_EFFORT depth=1: eski PC mesajları birikmez
_PC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

try:
    import torch
    _YOLO_DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
except ImportError:
    _YOLO_DEVICE = 'cpu'

def _imgmsg_to_numpy(msg):
    """cv_bridge gerektirmeden ROS Image mesajı -> numpy array."""
    return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1).copy()

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Ultralytics YOLO bulunamadı")

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..', '..', '..'))
_DEFAULT_MODEL = os.path.join(_PROJECT_ROOT, 'model', 'best.pt')


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Frame skip guard: YOLO inference süresi (~100-200ms) boyunca gelen yeni frame'ler atlanır
        self._busy = False
        self._busy_lock = threading.Lock()

        # Kamera görüntüsüne abone ol — BEST_EFFORT/depth=1: sadece en son frame
        self.subscription = self.create_subscription(
            Image,
            '/zed/image_raw',
            self.image_callback,
            _IMAGE_QOS
        )

        # ZED point cloud'a abone ol — BEST_EFFORT depth=1: hep son PC, eski mesajlar drop
        self.pc_subscription = self.create_subscription(
            Float32MultiArray,
            '/zed/point_cloud',
            self.point_cloud_callback,
            _PC_QOS
        )

        self.detection_publisher = self.create_publisher(String,  '/detection/objects',  10)
        self.distance_publisher  = self.create_publisher(Float32, '/detection/distance', 10)

        self.declare_parameter('model_path', _DEFAULT_MODEL)
        self.model = None

        # En son point cloud verisi: (H, W, 3) float32 array veya None
        self._point_cloud = None
        self._pc_step = 1  # downsample adımı (camera_node'dan gelir)
        self._pc_lock = threading.Lock()

        if YOLO_AVAILABLE:
            model_path = self.get_parameter('model_path').value
            try:
                self.model = YOLO(model_path)
                self.get_logger().info(f'YOLO modeli yüklendi (device: {_YOLO_DEVICE})')
                # TERMINAL: başlangıç logu
                print(f'[DETECTION] Başlatıldı | YOLO modeli yüklendi ({_YOLO_DEVICE})', flush=True)
            except Exception as e:
                self.get_logger().error(
                    f'YOLO modeli yüklenemedi: {e}\n'
                    f'Model dosyası bozuk olabilir. Modeli yeniden kopyalayın.')
                # TERMINAL: model hatası
                print(f'[DETECTION] ✗ Model yüklenemedi: {e}', flush=True)
                self.model = None
        else:
            self.get_logger().error('YOLO bulunamadı, nesne tespiti çalışmayacak.')
            print('[DETECTION] ✗ YOLO bulunamadı — nesne tespiti devre dışı', flush=True)
        # TERMINAL: 5s özet için sayaçlar
        import time as _time
        self._det_total_count = 0
        self._det_last_class = '-'
        self._det_last_summary = _time.time()
        # LOG DÜZENLEME: 5s özet timer kaldırıldı

    def point_cloud_callback(self, msg):
        """
        /zed/point_cloud formatı: [height, width, x0,y0,z0, x1,y1,z1, ...]
        Değerler santimetre cinsinden (ZED SDK CENTIMETER modunda).
        """
        try:
            data = msg.data
            if len(data) < 3:
                return
            h = int(data[0])
            w = int(data[1])
            step = int(data[2])  # downsample adımı
            # Tek np.array çağrısı: list → float32 array, sonra reshape (copy yok)
            raw = np.array(data[3:], dtype=np.float32)
            if raw.size == h * w * 3:
                with self._pc_lock:
                    self._point_cloud = raw.reshape(h, w, 3)
                    self._pc_step = step
        except Exception as e:
            self.get_logger().debug(f'Point cloud parse hatasi: {e}')

    def _get_distance_from_bbox(self, x1, y1, x2, y2):
        """
        Bounding box merkezindeki ZED point cloud değerinden 3D mesafe hesaplar.
        2024-2025 realsense.py mantığı: bbox içindeki min derinlik.
        Döndürür: mesafe cm cinsinden (float), ya da None.
        """
        with self._pc_lock:
            pc = self._point_cloud
            step = self._pc_step
        if pc is None:
            return None

        h, w = pc.shape[:2]
        # Orijinal piksel koordinatlarını downsample'a çevir
        cx = int((x1 + x2) / 2) // step
        cy = int((y1 + y2) / 2) // step

        # Bbox boyutuna göre örnekleme penceresi (downsample ölçeğinde)
        win = max(3, int(min(x2 - x1, y2 - y1) * 0.2) // step)
        rx1 = max(0, cx - win)
        rx2 = min(w, cx + win)
        ry1 = max(0, cy - win)
        ry2 = min(h, cy + win)

        bölge = pc[ry1:ry2, rx1:rx2]  # (ry, rx, 3)
        if bölge.size == 0:
            return None

        # Geçerli (sıfır olmayan, NaN olmayan) noktaları filtrele
        z_vals = bölge[:, :, 2].flatten()
        gecerli = z_vals[np.isfinite(z_vals) & (z_vals > 0)]
        if gecerli.size == 0:
            return None

        # XYZ Öklid mesafesi — merkez noktanın ortalaması üzerinden
        sample = bölge.reshape(-1, 3)
        mask = np.isfinite(sample).all(axis=1) & (sample[:, 2] > 0)
        if not mask.any():
            return None
        pts = sample[mask]
        dists = np.sqrt((pts ** 2).sum(axis=1))
        return float(dists.min())  # cm

    def image_callback(self, msg):
        """ROS spin thread: sadece guard kontrolü + thread spawn. Hiç bloklama yok."""
        if self.model is None:
            return
        with self._busy_lock:
            if self._busy:
                return
            self._busy = True
        # YOLO inference (~100-200ms) ayrı daemon thread'de — lane_detection ile aynı pattern
        threading.Thread(target=self._infer, args=(msg,), daemon=True).start()

    def _infer(self, msg):
        """Background thread: YOLO inference + publish. Spin thread serbest kalır."""
        try:
            frame = _imgmsg_to_numpy(msg)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            results = self.model.predict(
                frame_bgr, conf=0.5, device=_YOLO_DEVICE, imgsz=640, verbose=False)

            best_class = None
            best_conf = 0.0
            best_distance = 200.0

            for r in results:
                for box in r.boxes:
                    conf = box.conf[0].item()
                    if conf <= best_conf:
                        continue
                    cls = int(box.cls[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    best_conf = conf
                    best_class = self.model.names[cls] if self.model.names else str(cls)
                    best_distance = self._get_distance_from_bbox(x1, y1, x2, y2) or 200.0

            if best_class is not None:
                detection_msg = String()
                detection_msg.data = best_class
                self.detection_publisher.publish(detection_msg)

                distance_msg = Float32()
                distance_msg.data = best_distance
                self.distance_publisher.publish(distance_msg)

                if not hasattr(self, '_last_logged_class'):
                    self._last_logged_class = ''
                    self._same_class_count = 0

                if best_class != self._last_logged_class:
                    if self._same_class_count > 1:
                        self.get_logger().info(
                            f'Tespit özet: {self._last_logged_class} ({self._same_class_count}x tekrar)')
                    self.get_logger().info(
                        f'Tespit: {best_class} (conf: {best_conf:.2f}, mesafe: {best_distance:.1f} cm)')
                    # TERMINAL: tabela değişince bas
                    print(
                        f'[DETECTION] \u2713 Tabela: {best_class} | '
                        f'mesafe={best_distance/100:.1f}m | güven: %{best_conf*100:.0f}',
                        flush=True)
                    self._last_logged_class = best_class
                    self._same_class_count = 1
                else:
                    self._same_class_count += 1
                # TERMINAL: toplam sayım güncelle
                self._det_total_count += 1
                self._det_last_class = best_class
            else:
                # LOG DÜZENLEMETabela yok — sadece önceden bir tabela varken değişince bas
                if hasattr(self, '_last_logged_class') and self._last_logged_class != '':
                    print('[DETECTION] Tabela yok', flush=True)
                    self._last_logged_class = ''

        except Exception as e:
            self.get_logger().error(f'Object detection hatasi: {str(e)}')
        finally:
            with self._busy_lock:
                self._busy = False

def main(args=None):
    rclpy.init(args=args)
    detection_node = ObjectDetectionNode()

    try:
        rclpy.spin(detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        detection_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()