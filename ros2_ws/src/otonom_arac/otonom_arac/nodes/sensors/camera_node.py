import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
import time
import threading
import cv2
from otonom_arac.perf.metrics import FPSMeter, CallbackTimer, PerfPublisher

_IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
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
            self.runtime_params = sl.RuntimeParameters()

            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD720
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

        def grab_frame(self, want_point_cloud=False):
            err = self.zed.grab(self.runtime_params)
            if err == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image, sl.VIEW.LEFT, sl.MEM.CPU)
                image_numpy = self.image.get_data()
                image_rgb = cv2.cvtColor(image_numpy, cv2.COLOR_RGBA2RGB)
                point_cloud = None
                if want_point_cloud:
                    self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZ)
                    point_cloud = self.point_cloud
                return np.ascontiguousarray(image_rgb), point_cloud
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
            # Depth stream kaldırıldı — hiç kullanılmıyor, USB bant + CPU yakıyor
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(config)

        def get_frame(self):
            """Non-blocking tarzda: 33ms timeout (1 frame bütçesi). Hiç beklemez."""
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=33)
            except Exception:
                # Timeout veya hata: capture loop'u bloklama, None döndür
                return None
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None
            # np.asanyarray: zero-copy view — cv2.cvtColor kopyayı kendisi alır
            frame = np.asanyarray(color_frame.get_data())
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        def stop(self):
            self.pipeline.stop()

class LatestFrameHolder:
    """Thread-safe: Sadece en son frame'i tutar, eski frame'ler drop edilir."""
    def __init__(self):
        self._frame = None
        self._point_cloud = None
        self._capture_ns = None
        self._dropped = 0
        self._lock = threading.Lock()
    
    def put(self, frame, point_cloud=None, capture_ns=None):
        with self._lock:
            if self._frame is not None:
                self._dropped += 1
            self._frame = frame
            self._point_cloud = point_cloud
            self._capture_ns = capture_ns if capture_ns is not None else time.monotonic_ns()
    
    def get(self):
        """Frame'i al ve None'a sıfırla (consume)."""
        with self._lock:
            frame = self._frame
            pc = self._point_cloud
            capture_ns = self._capture_ns
            self._frame = None
            self._point_cloud = None
            self._capture_ns = None
            return frame, pc, capture_ns

    def dropped(self):
        with self._lock:
            return self._dropped


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self._cb_group = ReentrantCallbackGroup()
        # ZED publisher'lar — BEST_EFFORT/depth=1: GUI her zaman en son frame'i alır
        self.zed_publisher        = self.create_publisher(Image,            '/zed/image_raw',       _IMAGE_QOS)
        self.zed_preview_publisher = self.create_publisher(Image,            '/zed/preview',         _IMAGE_QOS)
        # depth=1 BEST_EFFORT: subscriber hep son point cloud'u alır, eski mesajlar drop edilir
        _PC_QOS = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.point_cloud_publisher = self.create_publisher(Float32MultiArray, '/zed/point_cloud', _PC_QOS)
        # RealSense publisher — BEST_EFFORT/depth=1
        self.realsense_publisher  = self.create_publisher(Image,            '/realsense/image_raw', _IMAGE_QOS)
        self.info = Info()

        # Perf ölçüm — /perf/summary topic'ini dinle: ros2 topic echo /perf/summary
        self._zed_cap_fps = FPSMeter('zed_cap')
        self._rs_cap_fps  = FPSMeter('rs_cap')
        self._zed_cb_timer = CallbackTimer('zed_cb', budget_ms=33.0)
        self._rs_cb_timer  = CallbackTimer('rs_cb',  budget_ms=33.0)
        self._pc_cb_timer  = CallbackTimer('pc_cb',  budget_ms=200.0)
        self._perf         = PerfPublisher(self, interval=2.0)
        self._perf.add_fps('zed_cap',  self._zed_cap_fps)
        self._perf.add_fps('rs_cap',   self._rs_cap_fps)
        self._perf.add_timer('zed_cb', self._zed_cb_timer)
        self._perf.add_timer('rs_cb',  self._rs_cb_timer)
        self._perf.add_timer('pc_cb',  self._pc_cb_timer)
        self._zed_pub_latency_ms = 0.0
        self._rs_pub_latency_ms = 0.0
        self._pc_age_ms = 0.0
        self._perf.add_value('zed_pub_age', lambda: f'{self._zed_pub_latency_ms:.1f}ms')
        self._perf.add_value('rs_pub_age', lambda: f'{self._rs_pub_latency_ms:.1f}ms')
        self._perf.add_value('pc_age', lambda: f'{self._pc_age_ms:.1f}ms')
        self._perf.add_value('zed_drop', lambda: str(self._zed_holder.dropped()))
        self._perf.add_value('rs_drop', lambda: str(self._rs_holder.dropped()))

        # Thread-safe frame holder'lar
        self._zed_holder = LatestFrameHolder()
        self._rs_holder = LatestFrameHolder()
        self._running = True
        self._pc_interval_ns = int(1e9 / 5)
        self._next_pc_capture_ns = 0

        # ZED başlat + capture thread
        self.camera = None
        self._zed_thread = None
        if CAMERA_AVAILABLE:
            try:
                self.camera = cam(self.info)
                self.get_logger().info('ZED kamera başlatıldı (ayrı thread).')
                self._zed_thread = threading.Thread(target=self._zed_capture_loop, daemon=True)
                self._zed_thread.start()
            except Exception as e:
                self.get_logger().error(f'ZED başlatılamadı: {str(e)}')
        else:
            self.get_logger().error('ZED SDK bulunamadı.')

        # RealSense başlat + capture thread
        self.realsense = None
        self._rs_thread = None
        if REALSENSE_AVAILABLE:
            try:
                self.realsense = RealsenseCam()
                self.get_logger().info('RealSense kamera başlatıldı (ayrı thread).')
                self._rs_thread = threading.Thread(target=self._rs_capture_loop, daemon=True)
                self._rs_thread.start()
            except Exception as e:
                self.get_logger().warn(f'RealSense başlatılamadı: {str(e)}')

        # Ayrı timer'lar — ReentrantCallbackGroup ile paralel çalışır
        self._zed_timer = self.create_timer(1.0/30, self._publish_zed_callback,
                                            callback_group=self._cb_group)
        self._rs_timer  = self.create_timer(1.0/30, self._publish_rs_callback,
                                            callback_group=self._cb_group)
        self._pc_timer  = self.create_timer(1.0/5,  self._publish_pc_callback,
                                            callback_group=self._cb_group)
        self._last_point_cloud = None  # PC timer için son point cloud'u sakla
        self._pc_data_lock = threading.Lock()

    def _zed_capture_loop(self):
        """ZED capture loop — kendi thread'inde sürekli çalışır."""
        while self._running and self.camera is not None:
            try:
                want_point_cloud = time.monotonic_ns() >= self._next_pc_capture_ns
                frame, pc = self.camera.grab_frame(want_point_cloud=want_point_cloud)
                if frame is not None:
                    capture_ns = time.monotonic_ns()
                    self._zed_cap_fps.tick()
                    self._zed_holder.put(frame, pc, capture_ns=capture_ns)
                    if want_point_cloud:
                        self._next_pc_capture_ns = capture_ns + self._pc_interval_ns
            except Exception:
                time.sleep(0.001)  # hata döngüsünde CPU spike engelle

    def _rs_capture_loop(self):
        """RealSense capture loop — kendi thread'inde sürekli çalışır."""
        while self._running and self.realsense is not None:
            try:
                frame = self.realsense.get_frame()
                if frame is not None:
                    capture_ns = time.monotonic_ns()
                    self._rs_cap_fps.tick()
                    self._rs_holder.put(frame, capture_ns=capture_ns)
            except Exception:
                time.sleep(0.001)  # hata döngüsünde CPU spike engelle

    def _publish_zed_callback(self):
        """30 Hz — ZED raw + preview yayınlar. PC'ye dokunmaz."""
        self._zed_cb_timer.start()
        try:
            zed_frame, point_cloud, capture_ns = self._zed_holder.get()
            if zed_frame is not None:
                if capture_ns is not None:
                    self._zed_pub_latency_ms = (time.monotonic_ns() - capture_ns) / 1e6
                stamp = self.get_clock().now().to_msg()

                # Tek resize + tek tobytes() — aynı msg nesnesi iki topic'e publish edilir.
                # HD720 (2.76 MB) yerine 640×360 (0.69 MB) serialize: ~4× daha az CPU/memory.
                # lane_detection w,h'yi frame'den alıyor, YOLO imgsz=640 iç resize yapıyor:
                # çözünürlük düşüşü algoritma doğruluğunu etkilemez.
                small = cv2.resize(zed_frame, (640, 360), interpolation=cv2.INTER_LINEAR)
                msg = _numpy_to_imgmsg(small, encoding='rgb8')
                msg.header.stamp = stamp
                msg.header.frame_id = 'zed_camera_link'
                self.zed_publisher.publish(msg)          # lane + object detection
                self.zed_preview_publisher.publish(msg)  # GUI — aynı nesne, ikinci kopya yok

                # En güncel point cloud'u PC timer için sakla
                if point_cloud is not None:
                    with self._pc_data_lock:
                        self._last_point_cloud = (point_cloud, capture_ns)
        except Exception as e:
            self.get_logger().error(f'ZED publish hatası: {str(e)}')
        finally:
            self._zed_cb_timer.stop()
            self._perf.tick()

    def _publish_rs_callback(self):
        """30 Hz — RealSense yayınlar. ZED'e dokunmaz."""
        self._rs_cb_timer.start()
        try:
            rs_frame, _, capture_ns = self._rs_holder.get()
            if rs_frame is not None:
                if capture_ns is not None:
                    self._rs_pub_latency_ms = (time.monotonic_ns() - capture_ns) / 1e6
                rs_msg = _numpy_to_imgmsg(rs_frame, encoding='rgb8')
                rs_msg.header.stamp = self.get_clock().now().to_msg()
                rs_msg.header.frame_id = 'realsense_camera_link'
                self.realsense_publisher.publish(rs_msg)
        except Exception as e:
            self.get_logger().error(f'RealSense publish hatası: {str(e)}')
        finally:
            self._rs_cb_timer.stop()
            self._perf.tick()

    def _publish_pc_callback(self):
        """5 Hz — Point cloud yayınlar. tolist() GIL spike ZED/RS'den izole."""
        self._pc_cb_timer.start()
        try:
            with self._pc_data_lock:
                pc_entry = self._last_point_cloud
                self._last_point_cloud = None
            if pc_entry is None:
                return
            pc, capture_ns = pc_entry
            if capture_ns is not None:
                self._pc_age_ms = (time.monotonic_ns() - capture_ns) / 1e6
            pc_data = pc.get_data()  # (H, W, 4) float32 — X,Y,Z,W cm
            STEP = 8
            ds = pc_data[::STEP, ::STEP, :3].astype(np.float32)
            h_ds, w_ds = ds.shape[:2]
            header = np.array([h_ds, w_ds, STEP], dtype=np.float32)
            combined = np.concatenate([header, ds.ravel()])
            pc_msg = Float32MultiArray()
            pc_msg.data = combined.tolist()
            self.point_cloud_publisher.publish(pc_msg)
        except Exception as e:
            self.get_logger().debug(f'Point cloud yayın hatası: {e}')
        finally:
            self._pc_cb_timer.stop()
            self._perf.tick()

    def destroy_node(self):
        """Node kapanırken thread'leri durdur."""
        self._running = False
        for t in (self._zed_thread, self._rs_thread):
            if t and t.is_alive():
                t.join(timeout=1.0)
        self._zed_timer.cancel()
        self._rs_timer.cancel()
        self._pc_timer.cancel()
        super().destroy_node()
    

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(camera_node)

    try:
        executor.spin()
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