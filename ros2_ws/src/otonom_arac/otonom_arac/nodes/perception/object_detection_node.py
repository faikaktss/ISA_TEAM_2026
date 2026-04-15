import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Float32MultiArray
import numpy as np

import cv2

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

        # Kamera görüntüsüne abone ol
        self.subscription = self.create_subscription(
            Image,
            '/zed/image_raw',
            self.image_callback,
            10
        )

        # ZED point cloud'a abone ol (gerçek mesafe için)
        self.pc_subscription = self.create_subscription(
            Float32MultiArray,
            '/zed/point_cloud',
            self.point_cloud_callback,
            10
        )

        self.detection_publisher = self.create_publisher(String,  '/detection/objects',  10)
        self.distance_publisher  = self.create_publisher(Float32, '/detection/distance', 10)

        self.declare_parameter('model_path', _DEFAULT_MODEL)
        self.model = None

        # En son point cloud verisi: (H, W, 3) float32 array veya None
        self._point_cloud = None
        self._pc_step = 1  # downsample adımı (camera_node'dan gelir)

        if YOLO_AVAILABLE:
            model_path = self.get_parameter('model_path').value
            try:
                self.model = YOLO(model_path)
                self.get_logger().info('YOLO modeli yüklendi')
            except Exception as e:
                self.get_logger().error(
                    f'YOLO modeli yüklenemedi: {e}\n'
                    f'Model dosyası bozuk olabilir. Modeli yeniden kopyalayın.')
                self.model = None
        else:
            self.get_logger().error('YOLO bulunamadı, nesne tespiti çalışmayacak.')

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
            xyz_flat = np.array(data[3:], dtype=np.float32)
            if xyz_flat.size == h * w * 3:
                self._point_cloud = xyz_flat.reshape(h, w, 3)
                self._pc_step = step
        except Exception as e:
            self.get_logger().debug(f'Point cloud parse hatasi: {e}')

    def _get_distance_from_bbox(self, x1, y1, x2, y2):
        """
        Bounding box merkezindeki ZED point cloud değerinden 3D mesafe hesaplar.
        2024-2025 realsense.py mantığı: bbox içindeki min derinlik.
        Döndürür: mesafe cm cinsinden (float), ya da None.
        """
        if self._point_cloud is None:
            return None

        h, w = self._point_cloud.shape[:2]
        step = self._pc_step
        # Orijinal piksel koordinatlarını downsample'a çevir
        cx = int((x1 + x2) / 2) // step
        cy = int((y1 + y2) / 2) // step

        # Bbox boyutuna göre örnekleme penceresi (downsample ölçeğinde)
        win = max(3, int(min(x2 - x1, y2 - y1) * 0.2) // step)
        rx1 = max(0, cx - win)
        rx2 = min(w, cx + win)
        ry1 = max(0, cy - win)
        ry2 = min(h, cy + win)

        bölge = self._point_cloud[ry1:ry2, rx1:rx2]  # (ry, rx, 3)
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
        if self.model is None:
            return
        try:
            frame = _imgmsg_to_numpy(msg)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            results = self.model.predict(frame_bgr, conf=0.5, device='cpu', imgsz=(736, 736))

            for r in results:
                for box in r.boxes:
                    cls  = int(box.cls[0])
                    conf = box.conf[0].item()
                    class_name = self.model.names[cls] if self.model.names else str(cls)

                    detection_msg = String()
                    detection_msg.data = class_name
                    self.detection_publisher.publish(detection_msg)

                    # Gerçek mesafe — ZED point cloud'dan
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    mesafe = self._get_distance_from_bbox(x1, y1, x2, y2)

                    distance_msg = Float32()
                    if mesafe is not None:
                        distance_msg.data = mesafe
                    else:
                        distance_msg.data = 200.0  # point cloud henüz gelmemişse varsayılan
                    self.distance_publisher.publish(distance_msg)

                    self.get_logger().info(
                        f'Tespit: {class_name} (conf: {conf:.2f}, mesafe: {distance_msg.data:.1f} cm)')

        except Exception as e:
            self.get_logger().error(f'Object detection hatasi: {str(e)}')


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
