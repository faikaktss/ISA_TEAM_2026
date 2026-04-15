import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import numpy as np
import math

import cv2

def _numpy_to_imgmsg(cv_image, encoding='rgb8'):
    """cv_bridge gerektirmeden numpy array -> ROS Image mesajı."""
    from sensor_msgs.msg import Image as ImageMsg
    msg = ImageMsg()
    msg.height, msg.width = cv_image.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = cv_image.shape[1] * cv_image.shape[2]
    msg.data = cv_image.tobytes()
    return msg


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

            err = self.zed.open(init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f'ZED açılamadı: {err}')

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
            return None, None

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

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError as e:
    REALSENSE_AVAILABLE = False
    print(f"RealSense kamera bulunamadı: {e}")

if REALSENSE_AVAILABLE:
    class RealsenseCam:
        def __init__(self):
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.pipeline.start(config)

        def get_frame(self):
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None
            frame = np.asanyarray(color_frame.get_data())
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        def stop(self):
            self.pipeline.stop()


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        # ZED publisher'lar
        self.zed_publisher        = self.create_publisher(Image,            '/zed/image_raw',   10)
        self.point_cloud_publisher = self.create_publisher(Float32MultiArray, '/zed/point_cloud', 10)
        # RealSense publisher
        self.realsense_publisher  = self.create_publisher(Image,            '/realsense/image_raw', 10)

        self.timer_period = 1.0/30 #Todo: 30 FPS
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        

        self.info = Info()

        # ZED başlat
        if CAMERA_AVAILABLE:
            try:
                self.camera = cam(self.info)
                self.get_logger().info('ZED kamera başlatıldı.')
            except Exception as e:
                self.get_logger().error(f'ZED başlatılamadı: {str(e)}')
                self.camera = None
        else:
            self.get_logger().error('ZED SDK bulunamadı.')
            self.camera = None

        # RealSense başlat
        self.realsense = None
        if REALSENSE_AVAILABLE:
            try:
                self.realsense = RealsenseCam()
                self.get_logger().info('RealSense kamera başlatıldı.')
            except Exception as e:
                self.get_logger().warn(f'RealSense başlatılamadı: {str(e)}')
        
    def timer_callback(self):
        try:
            # --- ZED ---
            zed_frame = None
            if self.camera is not None:
                zed_frame, point_cloud = self.camera.img_and_point_cloud()

            if zed_frame is not None:
                msg = _numpy_to_imgmsg(zed_frame, encoding='rgb8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'zed_camera_link'
                self.zed_publisher.publish(msg)

                # Point cloud yayınla: [height, width, x0,y0,z0, x1,y1,z1, ...]
                # Object detection node bunu bounding box merkezi için kullanır
                if point_cloud is not None:
                    try:
                        pc_data = point_cloud.get_data()  # (H, W, 4) fl oat32 — X,Y,Z,W cm
                        # 4x downsample: 1080x1920 → 270x480  (~25MB → ~1.5MB)
                        STEP = 4
                        ds = pc_data[::STEP, ::STEP, :3].astype(np.float32)
                        h_ds, w_ds = ds.shape[:2]
                        flat = ds.flatten().tolist()
                        pc_msg = Float32MultiArray()
                        pc_msg.data = [float(h_ds), float(w_ds), float(STEP)] + flat
                        self.point_cloud_publisher.publish(pc_msg)
                    except Exception as pc_err:
                        self.get_logger().debug(f'Point cloud yayın hatası: {pc_err}')

            # --- RealSense ---
            if self.realsense is not None:
                rs_frame = self.realsense.get_frame()
                if rs_frame is not None:
                    rs_msg = _numpy_to_imgmsg(rs_frame, encoding='rgb8')
                    rs_msg.header.stamp = self.get_clock().now().to_msg()
                    rs_msg.header.frame_id = 'realsense_camera_link'
                    self.realsense_publisher.publish(rs_msg)

        except Exception as e:
            self.get_logger().error(f'Görüntü işleme hatası: {str(e)}')
    

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()

    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        if camera_node.realsense is not None:
            camera_node.realsense.stop()
            print('RealSense kamera kapatıldı.')
        cv2.destroyAllWindows()
        camera_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()