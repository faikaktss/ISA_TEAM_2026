import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge 
import cv2
import numpy as np
import math


class Except(Exception):
    def __init__(self, message: str, error_code):
        super().__init__()
        self.message = message
        self.error_code = error_code
    
    def __str__(self):
        return f"{self.message}\nfonksiyon: {self.error_code}"


#Todo: Kameradan gelen görüntüden sadece yol kısmını alır ve şerit çizgilerinin kenarlarını ortaya çıkarır
class roi:
    def __init__(self, y1=400, y2=900, x1=500, x2=1400):
        self.y1 = y1
        self.y2 = y2
        self.x1 = x1
        self.x2 = x2

    #Todo: Roi boyutu gerçekten frame içine sığıyor mu Bunu kontrol eder. Kamera değil sadece yolu kontrol eder
    def get_roi(self, frame, width, height):
        if (self.x2 <= width and self.y2 <= height and self.x1 >= 0 and self.y1 >= 0 and self.x1 < self.x2 and self.y1 < self.y2):
            return frame[self.y1:self.y2, self.x1:self.x2]
        else:
            e = Except("roi ayarlanamadı", "get_roi()")
            print(e)
            print(f"Frame: {width}x{height}, ROI: x1={self.x1} x2={self.x2} y1={self.y1} y2={self.y2}")
            exit()

    #Todo: Şerit çizgilerinin kenarlarını ortaya çıkarmaya yarar
    def edge_detection(self, frame):
        # Griye çevir ve bulanıklaştır
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (25, 25), 0)

        # Todo: Şerit çizgileri beyaz asfalt siyah 
        mean_frame = cv2.mean(gray)[0]
        _, adaptive_thresh = cv2.threshold(
            blur,
            170,   #1.2*float(mean_frame),
            255,
            cv2.THRESH_BINARY
        )

        #Todo: siyah arka plan üzerinde beyaz çizgiler olur
        canny = cv2.Canny(adaptive_thresh, 50, 150)
        return canny


# Todo: Görüntüdeki değerin eğimini hesaplar
class angleCalculator:
    # Todo: Eğim bilgisi
    angle = None

    def find_slope(self, x1, y1, x2, y2):
        #Todo: Eğim i y'ye göre hesaplıyor
        if x2 - x1 != 0:
            return (x2 - x1) / (y2 - y1)
        else:
            print("Uyarı: Dikey doğru tespit edildi, eğim tanımsız.", "find_slope()")
            return None

    def slope_to_angle(self, slope):
        if slope is not None:
            #Todo: Eğimden açı hesaplanıyor
            angleCalculator.angle = -1 * math.atan(slope) * 180 / math.pi
            return angleCalculator.angle
        print("Açı hesaplanamadı.", "slope_to_angle()")
        return None


class midpointFinder(angleCalculator):
    def __init__(self):
        super().__init__()
        self.kayma = 0
        self.aci = 0

    #Todo: Orta noktayı bulan fonksiyon
    def find_midpoint(self, line1, line2):
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        midpoint_x = (x1 + x2 + x3 + x4) // 4
        midpoint_y = (y1 + y2 + y3 + y4) // 4

        slope1 = self.find_slope(x1, y1, x2, y2)
        slope2 = self.find_slope(x3, y3, x4, y4)

        angle1 = self.slope_to_angle(slope1)
        angle2 = self.slope_to_angle(slope2)

        return (midpoint_x, midpoint_y)


# Todo: Kuşbakışı görünüm dönüşümü
class birdEye:
    def __init__(self):
        self.pts1 = np.float32([(205, 329), (128, 476), (504, 330), (593, 476)])
        #Todo: Hedef dörtgen koordinatları
        self.pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])

    def transform(self, frame):
        #Todo: Görüntüyü 640x480 boyutuna getirir
        frame = cv2.resize(frame, (640, 480))
        #Todo: Perspektif dönüşüm matrisi hesaplanır
        matrix = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        #Todo: Perspektif dönüşümü uygulanır
        img = cv2.warpPerspective(frame, matrix, (640, 480))
        #Todo: Görüntüyü kırpar
        imgTrim = img[:360, :]
        return img


#Todo: Özel duraklar için kuşbakışı görünüm dönüşümü
class stationBirdEye:
    def __init__(self):
        self.pts1 = np.float32([(406, 348), (404, 478), (596, 335), (619, 477)])
        self.pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])
        self.transformed = None

    def transform(self, frame):
        frame = cv2.resize(frame, (640, 480))
        matrix = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        self.transformed = transformed = cv2.warpPerspective(frame, matrix, (640, 480))
        return transformed


class LaneDetection:
    def __init__(self, lane_position="Unknow"):
        self.roi_object = roi()
        self.angle_calculator = angleCalculator()
        self.midpoint_finder = midpointFinder()
        self.bird_eye = birdEye()
        self.lane_position = lane_position
        self.kayma = None
        self.midPointVectorAngle = None
        self.kaymaEsik = 60
        self.station_bird_eye = stationBirdEye()

    def kaymaEsigi_ve_VirajKontrol(self):
        if self.kayma is not None and self.kaymaEsik is not None:
            print("kaymahesaplaniyor")
            print(self.kaymaEsik < self.kaymaBuyukluk())
            return self.kaymaEsik < self.kaymaBuyukluk()
        else:
            return False

    #Todo: Kaymanının büyüklüğünü döner
    def kaymaBuyukluk(self):
        return abs(self.kayma)

    def process(self, frame, width, height):
        newroi = self.roi_object.get_roi(frame, width, height)
        edges = self.roi_object.edge_detection(newroi)
        mask = edges.copy()

        pts = np.array([[186, 700], [453, 500], [940, 494], [1144, 760]], dtype=np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.polylines(edges, [pts], True, (0, 0, 0), 10)
        cv2.fillPoly(mask, [pts], (0, 0, 0))
        result = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=50)
        result = np.copy(newroi)

        left_lines = []
        right_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = self.angle_calculator.find_slope(x1, y1, x2, y2)
                if slope is not None:
                    if slope < 0:
                        left_lines.append((x1, y1, x2, y2))
                    elif slope > 0:
                        right_lines.append((x1, y1, x2, y2))
                cv2.line(result, (x1, y1), (x2, y2), (255, 0, 0), thickness=5)

        steering_angle = 0
        self.lane_position = "Unknown"

        if left_lines and right_lines:
            left_avg = np.mean(left_lines, axis=0, dtype=int)
            right_avg = np.mean(right_lines, axis=0, dtype=int)

            midpoint_x, midpoint_y = self.midpoint_finder.find_midpoint(left_avg, right_avg)
            x, y = edges.shape
            self.kayma = x - midpoint_x
            self.midPointVectorAngle = math.atan((x - midpoint_x) / (midpoint_y)) * ((180 / math.pi))
            print(f"{self.midPointVectorAngle} ve kayma : {self.kayma}")
            print(self.kayma)
            cv2.circle(result, (midpoint_x, midpoint_y), 10, (0, 255, 0), -1)
            cv2.imshow("orta", result)

            frame_center = (newroi.shape[1]) // 2
            lane_center = (left_avg[2] + right_avg[2]) // 2
            steering_angle = (lane_center - frame_center) * 0.1

            if abs(self.kayma) < 10:
                if lane_center < frame_center - 50:
                    self.lane_position = -1
                elif lane_center > frame_center + 50:
                    self.lane_position = 1
                else:
                    self.lane_position = 0

            self.kayma = midpoint_x - x

        cv2.putText(result, f"Angle: {int(steering_angle)} degrees", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(result, f"Lane: {self.lane_position}", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        return result, edges

    def bird_eye_view(self, frame):
        bird = self.bird_eye.transform(frame)
        bird_canny = self.roi_object.edge_detection(bird)
        bird_lines = cv2.HoughLinesP(bird_canny, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=50)

        angle = None

        if bird_lines is not None:
            for line in bird_lines:
                x1, y1, x2, y2 = line[0]
                slope = self.angle_calculator.find_slope(x1, y1, x2, y2)
                angle = self.angle_calculator.slope_to_angle(slope)
                cv2.line(bird, (x1, y1), (x2, y2), (0, 0, 255), thickness=5)

        return bird, bird_canny, angle

    def station_bird_eye_view(self, frame):
        station_bird = self.station_bird_eye.transform(frame)
        station_edges = self.roi_object.edge_detection(station_bird)
        station_lines = cv2.HoughLinesP(station_edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=50)

        angle = None
        if station_lines is not None:
            for line in station_lines:
                x1, y1, x2, y2 = line[0]
                slope = self.angle_calculator.find_slope(x1, y1, x2, y2)
                angle = self.angle_calculator.slope_to_angle(slope)
                cv2.line(station_bird, (x1, y1), (x2, y2), (0, 0, 255), thickness=5)

        return station_bird, angle



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
        self.lane_detector = LaneDetection()
        # ROI ayarları (1920x1080 için)
        self.lane_detector.roi_object.y1 = 400
        self.lane_detector.roi_object.y2 = 900
        self.lane_detector.roi_object.x1 = 500
        self.lane_detector.roi_object.x2 = 1400
        self.get_logger().info('Şerit algılama başlatıldı')
        
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