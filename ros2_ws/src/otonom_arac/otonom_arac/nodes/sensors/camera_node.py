import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


# Todo: Robotun tüm sensör ve verilerini saklamak için kullanılan bilgi sınıfı
class Info:
    line = None
    def __init__(self):
        self._angle = None
        self._line = None
        self._tabela = None
        self._encoder = None
        self._imu = None
        self._gps = None
        self._name = None
        self._description = None
        self._lane = None
        self._originalFrame = None
        self._birdEyeFrame = None
        self._distance = None
        self._engel = None
        self._edgeBirdEye = None
        self.durakPixel = 0
        self._solRedCount = None
        self._sagRedCount = None
        self._durak_engel = None

    def set_angle(self, angle):
        self._angle = angle

    def get_angle(self):
        return self._angle

    def set_line(self, line):
        self._line = line
        
    def get_line(self):
        return self._line

    def set_tabela(self, tabela):
        self._tabela = tabela
    
    def get_tabela(self):
        return self._tabela
    
    def set_encoder(self, encoder):
        self._encoder = encoder

    def get_encoder(self):
        return self._encoder
    
    def set_imu(self, imu):
        self._imu = imu

    def get_imu(self):
        return self._imu
    
    def set_gps(self, gps):
        self._gps = gps
    
    def get_gps(self):
        return self._gps
    
    def set_name(self, name):
        self._name = name
    
    def get_name(self):
        return self._name
    
    def set_description(self, description):
        self._description = description
    
    def get_description(self):
        return self._description
    
    def set_lane(self, lane):
        self._lane = lane
    
    def get_lane(self):
        return self._lane
    
    def set_originalFrame(self, originalFrame):
        self._originalFrame = originalFrame
    
    def get_originalFrame(self):
        return self._originalFrame
    
    def set_birdEyeFrame(self, birdEyeFrame):
        self._birdEyeFrame = birdEyeFrame
    
    def get_birdEyeFrame(self):
        return self._birdEyeFrame
    
    def set_distance(self, distance):
        self._distance = distance
    
    def get_distance(self):
        return self._distance
    
    def set_engel(self, engel):
        self._engel = engel
    
    def get_engel(self):
        return self._engel
        
    def set_edgeBirdEye(self, edgeBirdEye):
        self._edgeBirdEye = edgeBirdEye
    
    def get_edgeBirdEye(self):
        return self._edgeBirdEye
    
    def set_durakPixel(self, v):
        self.durakPixel = v

    def get_durakPixel(self):
        return self.durakPixel

    def set_solRedCount(self, solRedCount):
        self._solRedCount = solRedCount
    
    def get_solRedCount(self):
        return self._solRedCount
    
    def set_sagRedCount(self, sagRedCount):
        self._sagRedCount = sagRedCount
    
    def get_sagRedCount(self):
        return self._sagRedCount
    
    def set_durak_engel(self, durak_engel):
        self._durak_engel = durak_engel

    def get_durak_engel(self):
        return self._durak_engel

try:
    import pyzed.sl as sl

    #Todo: ZED kamera ile görüntü yakalama ve nokta bulutu işleme sınıfı
    class cam:
        def __init__(self, info: Info):
            self.zed = sl.Camera()
            self.image = sl.Mat()
            self.point_cloud = sl.Mat()
            self.depth = sl.Mat()
            self.info = info

            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD1080
            init_params.camera_fps = 30
            init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
            init_params.coordinate_units = sl.UNIT.CENTIMETER

            self.zed.open(init_params)

            cam_info = self.zed.get_camera_information()
            resolution = cam_info.camera_configuration.resolution
            self.width = resolution.width
            self.height = resolution.height

        def img_and_point_cloud(self):
            err = self.zed.grab()
            if err == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
                image_numpy = self.image.get_data()
                self.img = cv2.cvtColor(image_numpy, cv2.COLOR_RGBA2RGB)

                self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZ)
                return self.img, self.point_cloud
            print("zed grab() hatası.", err)
            exit()

        def get_distance(self, point_cloud, x, y):
            err, point_cloud_value = point_cloud.get_value(x, y)
            if err == sl.ERROR_CODE.SUCCESS:
                x3d, y3d, z3d = point_cloud_value[0], point_cloud_value[1], point_cloud_value[2]
                if x3d == 0 and y3d == 0 and z3d == 0:
                    return float('inf')
                return math.sqrt(x3d**2 + y3d**2 + z3d**2)
            else:
                return float('inf')

    CAMERA_AVAILABLE = True
except ImportError as e:
    CAMERA_AVAILABLE = False
    print(f" ZED kamera bulunamadı: {e}")


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
                self.get_logger().info('ZED kamera başlatıldı.')
            except Exception as e:
                self.get_logger().warn(f' ZED başlatılamadı: {str(e)}. Test moduna geçiliyor.')
                self.test_mode = True
        else:
            self.get_logger().warn(' ZED SDK bulunamadı. Test modu aktif.')
            self.test_mode = True
        
        if self.test_mode:
            self.frame_counter = 0
            self.get_logger().info(' Test kamerası hazır (1920x1080 sahte görüntü)')
        
    def timer_callback(self):
        try:
            #Todo: Sahte görüntü üret veya gerçek kamera görüntüsü al
            if self.test_mode:
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

                cv2.imshow('Kamera', frame)
                cv2.waitKey(1)

        except SystemExit:
            self.get_logger().info('Kamera bağlantısı koptu')
            self.destroy_node()
        except Exception as e:
            self.get_logger().error(f'Görüntü işleme hatası: {str(e)}')
    

    #Todo: Test aşaması için sahte görüntü üret(faik)
    def _generate_test_frame(self):
        self.frame_counter += 1
        
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        
        for y in range(480):
            frame[y, :] = [y // 2, 100, 255 - y // 2]
        
        cv2.putText(frame, f'TEST MODE - Frame: {self.frame_counter}', 
            (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, 'ROS 2 Camera Node Active', 
            (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        #Todo: Dinamik bir nokta ekle(faik)
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
        cv2.destroyAllWindows()
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()