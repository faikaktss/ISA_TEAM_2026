import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from .camera import cam
from .info import Info

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer_period = 1.0/30 #Todo: 30 FPS
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.bridge = CvBridge()

        self.info = Info()
        try:
            self.camera = cam(self.info)
            self.get_logger().info('Zed kamera başlatıldı.')
        except Exception as e:
            self.get_logger().error(f'Kamera başlatılamadı: {str(e)}')
            raise e
        
    def timer_callback(self):
        try:
            #Todo: Kameradan görüntü al ve nokta bulutunu al
            frame, point_cloud = self.camera.img_and_point_cloud()

            if frame is not None:
                #Todo: OpenCV görüntüsünü ROS Image mesajına dönüştür
                msg = self.bridge.cv2_to_imgmsg(frame,encoding='rgb8')
                msg.header.stamp = self.get_clock().now().to_msg()#Todo: Zaman damgası ekle
                msg.header.frame_id = 'zed_camera_link' #Todo: Çerçeve kimliği ekle

                self.publisher.publish(msg)

        except SystemExit:
            self.get_logger().info('Kamera bağlantısı koptu')
            self.destroy_node()
        except Exception as e:
            self.get_logger().error(f'Görüntü işleme hatası: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()

    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass:
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()