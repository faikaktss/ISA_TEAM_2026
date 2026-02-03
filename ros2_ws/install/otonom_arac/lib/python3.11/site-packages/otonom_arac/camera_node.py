import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from .info import Info

# ZED kamera import'unu güvenli hale getir
try:
    from .camera import cam
    CAMERA_AVAILABLE = True
except ImportError as e:
    CAMERA_AVAILABLE = False
    print(f"⚠️  ZED kamera bulunamadı: {e}")
    print("📷 Test modu aktif - Sahte görüntü üretiliyor")

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer_period = 1.0/30 #Todo: 30 FPS
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.bridge = CvBridge()

        self.info = Info()
        self.test_mode = False
        
        if CAMERA_AVAILABLE:
            try:
                self.camera = cam(self.info)
                self.get_logger().info('✅ ZED kamera başlatıldı.')
            except Exception as e:
                self.get_logger().warn(f'⚠️  ZED başlatılamadı: {str(e)}. Test moduna geçiliyor.')
                self.test_mode = True
        else:
            self.get_logger().warn('⚠️  ZED SDK bulunamadı. Test modu aktif.')
            self.test_mode = True
        
        if self.test_mode:
            self.frame_counter = 0
            self.get_logger().info('📷 Test kamerası hazır (640x480 sahte görüntü)')
        
    def timer_callback(self):
        try:
            if self.test_mode:
                # Test modu: Sahte görüntü üret
                frame = self._generate_test_frame()
            else:
                #Todo: Kameradan görüntü al ve nokta bulutunu al
                frame, point_cloud = self.camera.img_and_point_cloud()

            if frame is not None:
                #Todo: OpenCV görüntüsünü ROS Image mesajına dönüştür
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
                msg.header.stamp = self.get_clock().now().to_msg()#Todo: Zaman damgası ekle
                msg.header.frame_id = 'camera_link' #Todo: Çerçeve kimliği ekle

                self.publisher.publish(msg)

        except SystemExit:
            self.get_logger().info('Kamera bağlantısı koptu')
            self.destroy_node()
        except Exception as e:
            self.get_logger().error(f'Görüntü işleme hatası: {str(e)}')
    
    def _generate_test_frame(self):
        """Test için sahte görüntü üret"""
        self.frame_counter += 1
        
        # 640x480 RGB görüntü
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Arka plan gradyanı
        for y in range(480):
            frame[y, :] = [y // 2, 100, 255 - y // 2]
        
        # Test metni
        cv2.putText(frame, f'TEST MODE - Frame: {self.frame_counter}', 
                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, 'ROS 2 Camera Node Active', 
                    (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Hareket eden daire
        cx = int(320 + 200 * np.sin(self.frame_counter * 0.1))
        cy = int(240 + 100 * np.cos(self.frame_counter * 0.1))
        cv2.circle(frame, (cx, cy), 30, (0, 255, 255), -1)
        
        return frame


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()

    try:
        rclpy.spin(camera_node)#Todo: timer çalıştığı sürece programı ayakta tutar
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()