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


class roi:
    def __init__(self, y1=400, y2=900, x1=500, x2=1400):
        self.y1 = y1
        self.y2 = y2
        self.x1 = x1
        self.x2 = x2

        # Trapez köşe noktaları — TAM FRAME koordinatlarında
        # Sol taraf daha dar (içe çekilmiş), sağ taraf geniş
        # Hedef: sağ şerit + orta kesikli çizgi görünsün
        # Araç ROI dışında kalır (alt sınır y=510)
        # Üst sınır daha yukarı çekildi (y=200)
        #
        #   sol_üst(430,200) ----------- sağ_üst(1100,200)
        #       /                                       \
        # sol_alt(150,510) -------------------- sağ_alt(1280,510)
        #
        self.trapez_sol_alt  = (150,  510)
        self.trapez_sol_ust  = (430,  200)
        self.trapez_sag_ust  = (1100, 200)
        self.trapez_sag_alt  = (1280, 510)

    def get_roi(self, frame, width, height):
        if (self.x2 <= width and self.y2 <= height and self.x1 >= 0 and self.y1 >= 0 and self.x1 < self.x2 and self.y1 < self.y2):
            return frame[self.y1:self.y2, self.x1:self.x2]
        else:
            e = Except("roi ayarlanamadı", "get_roi()")
            print(e)
            print(f"Frame: {width}x{height}, ROI: x1={self.x1} x2={self.x2} y1={self.y1} y2={self.y2}")
            exit()

    def apply_trapez_mask(self, frame):
        """
        Tam frame üzerine trapez maske uygular.
        Trapez dışındaki her yer siyaha çekilir.
        Sadece iki şeridin geçtiği alan görünür kalır.
        """
        pts = np.array([
            list(self.trapez_sol_alt),
            list(self.trapez_sol_ust),
            list(self.trapez_sag_ust),
            list(self.trapez_sag_alt),
        ], dtype=np.int32)

        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 255)

        if len(frame.shape) == 3:
            mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            masked = cv2.bitwise_and(frame, mask_3ch)
        else:
            masked = cv2.bitwise_and(frame, mask)

        return masked, mask

    def edge_detection(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (25, 25), 0)

        # Otsu: her karede eşik değerini otomatik hesaplar
        _, adaptive_thresh = cv2.threshold(
            blur,
            0,
            255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

        canny = cv2.Canny(adaptive_thresh, 50, 150)
        return canny


class angleCalculator:
    angle = None

    def find_slope(self, x1, y1, x2, y2):
        if x2 - x1 != 0:
            return (x2 - x1) / (y2 - y1)
        else:
            print("Uyarı: Dikey doğru tespit edildi, eğim tanımsız.", "find_slope()")
            return None

    def slope_to_angle(self, slope):
        if slope is not None:
            angleCalculator.angle = -1 * math.atan(slope) * 180 / math.pi
            return angleCalculator.angle
        print("Açı hesaplanamadı.", "slope_to_angle()")
        return None


class midpointFinder(angleCalculator):
    def __init__(self):
        super().__init__()
        self.kayma = 0
        self.aci = 0

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


class birdEye:
    def __init__(self):
        self.pts1 = np.float32([(205, 329), (128, 476), (504, 330), (593, 476)])
        self.pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])

    def transform(self, frame):
        frame = cv2.resize(frame, (640, 480))
        matrix = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        img = cv2.warpPerspective(frame, matrix, (640, 480))
        imgTrim = img[:360, :]
        return img


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

        # Temporal smoothing
        self.smoothed_angle = 0.0
        self.smoothed_kayma = 0.0
        self.smoothing_alpha = 0.3

    def kaymaEsigi_ve_VirajKontrol(self):
        if self.kayma is not None and self.kaymaEsik is not None:
            print("kaymahesaplaniyor")
            print(self.kaymaEsik < self.kaymaBuyukluk())
            return self.kaymaEsik < self.kaymaBuyukluk()
        else:
            return False

    def kaymaBuyukluk(self):
        return abs(self.kayma)

    def process(self, frame, width, height):
        # 1. Adım: Önce tam frame'e trapez maske uygula
        masked_frame, trapez_mask = self.roi_object.apply_trapez_mask(frame)

        # 2. Adım: Maskelenmiş frame'den kenar tespiti yap
        edges = self.roi_object.edge_detection(masked_frame)

        # 3. Adım: Hough ile çizgi tespiti — artık ROI kesmiyoruz,
        #          trapez maske zaten gereksiz bölgeleri siyaha çekti
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=50)
        result = masked_frame.copy()

        # Trapez sınırını turuncu çiz (debug)
        trap_pts = np.array([
            list(self.roi_object.trapez_sol_alt),
            list(self.roi_object.trapez_sol_ust),
            list(self.roi_object.trapez_sag_ust),
            list(self.roi_object.trapez_sag_alt),
        ], dtype=np.int32)
        cv2.polylines(result, [trap_pts.reshape((-1,1,2))], True, (0, 165, 255), 2)

        left_lines = []
        right_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = self.angle_calculator.find_slope(x1, y1, x2, y2)

                # Eğim büyüklük filtresi: yatay ve dikey gürültüler elenir
                if slope is not None:
                    if -2.5 < slope < -0.3:
                        left_lines.append((x1, y1, x2, y2))
                    elif 0.3 < slope < 2.5:
                        right_lines.append((x1, y1, x2, y2))

                cv2.line(result, (x1, y1), (x2, y2), (255, 0, 0), thickness=5)

        steering_angle = 0
        self.lane_position = "Unknown"

        if left_lines and right_lines:
            left_avg = np.mean(left_lines, axis=0, dtype=int)
            right_avg = np.mean(right_lines, axis=0, dtype=int)

            midpoint_x, midpoint_y = self.midpoint_finder.find_midpoint(left_avg, right_avg)

            # Kayma: frame genişliğinin merkezine göre
            frame_center_x = width // 2
            self.kayma = midpoint_x - frame_center_x

            if midpoint_y != 0:
                self.midPointVectorAngle = math.atan(self.kayma / midpoint_y) * (180 / math.pi)
            else:
                self.midPointVectorAngle = 0.0

            print(f"{self.midPointVectorAngle} ve kayma : {self.kayma}")

            # Sol şerit kırmızı, sağ şerit sarı
            for lx1, ly1, lx2, ly2 in left_lines:
                cv2.line(result, (lx1, ly1), (lx2, ly2), (0, 0, 255), thickness=3)
            for rx1, ry1, rx2, ry2 in right_lines:
                cv2.line(result, (rx1, ry1), (rx2, ry2), (0, 255, 255), thickness=3)

            # Midpoint yeşil nokta
            cv2.circle(result, (midpoint_x, midpoint_y), 12, (0, 255, 0), -1)

            # Frame merkez çizgisi beyaz
            cv2.line(result, (frame_center_x, 0), (frame_center_x, height), (255, 255, 255), 1)

            lane_center = (left_avg[2] + right_avg[2]) // 2
            steering_angle = (lane_center - frame_center_x) * 0.1

            # Lane position
            if abs(self.kayma) >= 10:
                if lane_center < frame_center_x - 50:
                    self.lane_position = -1
                elif lane_center > frame_center_x + 50:
                    self.lane_position = 1
                else:
                    self.lane_position = 0
            else:
                self.lane_position = 0

            # Temporal smoothing
            self.smoothed_angle = (
                self.smoothing_alpha * self.midPointVectorAngle
                + (1 - self.smoothing_alpha) * self.smoothed_angle
            )
            self.smoothed_kayma = (
                self.smoothing_alpha * self.kayma
                + (1 - self.smoothing_alpha) * self.smoothed_kayma
            )

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

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.angle_publisher = self.create_publisher(Float32, '/lane/angle', 10)
        self.offset_publisher = self.create_publisher(Float32, '/lane/offset', 10)
        self.debug_image_publisher = self.create_publisher(Image, '/lane/debug_image', 10)

        self.bridge = CvBridge()
        self.lane_detector = LaneDetection()

        # Trapez köşe noktaları — tam frame koordinatları (1280x720)
        #
        #   (430,200) ------------ (1100,200)
        #      /                            \
        # (150,510) -------------------- (1280,510)
        #
        # Sol taraf daha dar → sol şerit + orta kesikli çizgi görünür
        # Sağ taraf geniş → sağ şerit tam kapsanır
        # Alt sınır y=510 → araç ROI dışında kalır
        # Üst sınır y=200 → daha fazla yol görünür
        self.lane_detector.roi_object.trapez_sol_alt = (150,  510)
        self.lane_detector.roi_object.trapez_sol_ust = (430,  200)
        self.lane_detector.roi_object.trapez_sag_ust = (1100, 200)
        self.lane_detector.roi_object.trapez_sag_alt = (1280, 510)

        self.get_logger().info('Şerit algılama başlatıldı')
        self.get_logger().info('Trapez ROI: sol_alt(150,510) sol_ust(430,200) sag_ust(1100,200) sag_alt(1280,510)')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            height, width = frame_bgr.shape[:2]
            result, edges = self.lane_detector.process(frame_bgr, width, height)

            if result is not None:
                result_rgb = cv2.cvtColor(result, cv2.COLOR_BGR2RGB)
                debug_msg = self.bridge.cv2_to_imgmsg(result_rgb, encoding='rgb8')
                self.debug_image_publisher.publish(debug_msg)

            # Yumuşatılmış değerler yayınlanır
            if self.lane_detector.midPointVectorAngle is not None:
                angle_msg = Float32()
                angle_msg.data = float(self.lane_detector.smoothed_angle)
                self.angle_publisher.publish(angle_msg)

            if self.lane_detector.kayma is not None:
                offset_msg = Float32()
                offset_msg.data = float(self.lane_detector.smoothed_kayma)
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
