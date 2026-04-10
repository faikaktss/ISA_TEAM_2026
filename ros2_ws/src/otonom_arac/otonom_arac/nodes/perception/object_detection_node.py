import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

#Todo:YOLO modelini import et
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
        
        #Todo: Kamera görüntüsüne abone ol
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        #Todo:Tespit edilen nesneleri yayınla
        self.detection_publisher = self.create_publisher(String, '/detection/objects', 10)
        self.distance_publisher = self.create_publisher(Float32, '/detection/distance', 10)
        
        self.bridge = CvBridge()

        self.declare_parameter('model_path', _DEFAULT_MODEL)

        if YOLO_AVAILABLE:
            #Todo: YOLO modelini yükle
            model_path = self.get_parameter('model_path').value
            self.model = YOLO(model_path)
            self.get_logger().info('YOLO modeli yüklendi')
        else:
            self.get_logger().error('YOLO bulunamadı, nesne tespiti çalışmayacak.')
            self.model = None
    
    def image_callback(self, msg):
        try:
            #Todo : ROS Image 'ten OpenCV'ye dönüştürür
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            if self.model is not None:
                #Todo:Gerçek YOLO tespiti
                results = self.model.predict(frame_bgr, conf=0.5, device='cpu', imgsz=(736, 736))
                
                for r in results:
                    for box in r.boxes:
                        cls = int(box.cls[0])
                        conf = box.conf[0].item()
                        
                        if conf > 0.5:
                            class_name = self.model.names[cls] if self.model.names else str(cls)
                            
                            #Todo: Nesne tespiti yayınla
                            detection_msg = String()
                            detection_msg.data = class_name
                            self.detection_publisher.publish(detection_msg)
                            
                            #Todo: Mesafe hesaplama (nokta bulutu gerekli - şimdilik sabit)
                            distance_msg = Float32()
                            distance_msg.data = 200.0  # Placeholder
                            self.distance_publisher.publish(distance_msg)
                            
                            self.get_logger().info(f'Tespit: {class_name} (conf: {conf:.2f})')
                
        except Exception as e:
            self.get_logger().error(f'Object detection hatası: {str(e)}')

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