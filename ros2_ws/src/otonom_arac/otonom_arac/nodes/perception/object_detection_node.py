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
        self.test_mode = False

        self.declare_parameter('model_path', '/home/user/model/best.pt')

        if YOLO_AVAILABLE:
            try:
                #Todo: YOLO modelini yükle
                model_path = self.get_parameter('model_path').value
                self.model = YOLO(model_path)
                self.get_logger().info('YOLO modeli yüklendi')
            except Exception as e:
                self.get_logger().warn(f'YOLO yüklenemedi: {str(e)}. Test modu.')
                self.test_mode = True
        else:
            self.get_logger().warn('YOLO bulunamadı. Test modu aktif.')
            self.test_mode = True
        
        if self.test_mode:
            self.frame_count = 0
            self.get_logger().info('Test object detection hazır')
    
    def image_callback(self, msg):
        try:
            #Todo : ROS Image 'ten OpenCV'ye dönüştürür
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            if self.test_mode:
                # Todo Test: Her 30 frame'de bir "kirmizi" tabelası tespit et
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    detection_msg = String()
                    detection_msg.data = "kirmizi"
                    self.detection_publisher.publish(detection_msg)
                    
                    distance_msg = Float32()
                    distance_msg.data = 150.0  # 150 cm
                    self.distance_publisher.publish(distance_msg)
                    
                    self.get_logger().info('Test: kirmizi tabela tespit edildi (150cm)')
            else:
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()