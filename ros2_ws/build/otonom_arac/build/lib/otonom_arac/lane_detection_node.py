import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge 
import cv2
import numpy as np

#Todo: LaneDetect modülünü içe aktarmaya çalış
try:
    from .LaneDetect import LaneDetection
    LANE_DETECT_AVAILABLE = True
except ImportError as e:
    LANE_DETECT_AVAILABLE =False
    print(f"LaneDetect import hatası: {e}")



class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        #Todo: Bir abone oluştur
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        #Todo: Şerit verilerini yayınlar
        self.angle_publisher = self.create_publisher(Float32,'/lane/angle',10)
        self.offset_publisher = self.create_publisher(Float32,'/lane/offset',10)


        #Todo:Ros opencv dönüşümü
        self.bridge = CvBridge()
        if LANE_DETECT_AVAILABLE:
            self.lane_detector = LaneDetection()
            # ROI ayarları (1920x1080 için)
            self.lane_detector.roi_object.y1 = 400
            self.lane_detector.roi_object.y2 = 900
            self.lane_detector.roi_object.x1 = 500
            self.lane_detector.roi_object.x2 = 1400
            self.get_logger().info('Şerit algılama başlatıldı')
        else:
            self.get_logger().error('LaneDetection modülü bulunamadı. Şerit algılama devre dışı bırakıldı.')
            raise ImportError('LaneDetect gerekli')
        
    def image_callback(self,msg):
        try:
            #Todo: ROS Image mesajını OpenCV görüntüsüne dönüştür
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            #Todo: Şerit algılama yap
            height, width = frame_bgr.shape[:2]
            result, edges = self.lane_detector.process(frame_bgr, width, height)

            #Todo: Açı ve kayma verilerini yayınla
            if self.lane_detector.midPointVectorAngle is not None:
                angle_msg = Float32()
                angle_msg.data = float(self.lane_detector.midPointVectorAngle)
                self.angle_publisher.publish(angle_msg)
            #Todo: Sapma verisini yayınla
            if self.lane_detector.kayma is not None:
                offset_msg = Float32()
                offset_msg.data = float(self.lane_detector.kayma)
                self.offset_publisher.publish(offset_msg)

        except Exception as e:
            self.get_logger().error(f'Görüntü işleme hatası: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    lane_node = LaneDetectionNode()

    try:
        rclpy.spin(lane_node)
    except KeyboardInterrupt:
        pass
    finally:
        lane_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()