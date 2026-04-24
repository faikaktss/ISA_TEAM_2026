import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
import time
import threading
import gc
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
            # VGA (672x376): ham frame 1280x720x4=3.7MB yerine 672x376x4=1.01MB
            # Grab süresi azalır, latency düşer. Pipeline çıkışı 640x360 olarak sabit kalır.
            init_params.camera_resolution = sl.RESOLUTION.VGA
            init_params.camera_fps = 30
            # DEPTH_MODE.NONE: her grab'da stereo derinlik hesabı YAPILMAZ.
            # 30Hz'de ~8-15ms/frame tasarruf. point cloud ihtiyacı olunca
            # runtime_params.enable_depth = True ile anında etkinleştirilebilir.
            init_params.depth_mode = sl.DEPTH_MODE.NONE
            init_params.coordinate_units = sl.UNIT.CENTIMETER

            # Point cloud çıktısı için ayrı ZED instance (opsiyonel, şimdilik devre dışı)
            self._depth_enabled = False

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
                # ZED SDK gerçek capture timestamp'ini al (UNIX nanoseconds)
                ts = self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)
                frame_ns = ts.get_nanoseconds()
                # FIX SORUN-2: stale drain kaldırıldı — çift grab() 30fps→~15fps jitter yaratıyordu.
                self.zed.retrieve_image(self.image, sl.VIEW.LEFT, sl.MEM.CPU)
                image_numpy = self.image.get_data()
                # BUG-A FIX: cvtColor T1'den KALDIRILDI.
                # Önceden: grab() biter → T1 cvtColor için 0.5ms GIL tutardı → T3 ve T2 beklerdi.
                # Şimdi: ham RGBA array döndür, cvtColor T2 (_zed_publish_loop) içinde yapılır.
                # T1'in grab() sonrası GIL süresi ~0.5ms → ~0.05ms'ye düştü.
                # np.ascontiguousarray: ZED Mat'ı C-contiguous yap — cvtColor için gerekli.
                return np.ascontiguousarray(image_numpy), None, frame_ns
            return None, None, None

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
            """30fps=33ms frame süresi; timeout 33ms ile frame gelene kadar blokla."""
            try:
                # FIX SORUN-3: timeout_ms=5 iken 30fps'de %85 timeout oluyordu →
                # CPU spin + frame kaybi. 33ms ile her dongu iterasyonu 1 frame uretir.
                frames = self.pipeline.wait_for_frames(timeout_ms=33)
            except Exception:
                return None
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None
            # np.asanyarray: zero-copy view — cv2.cvtColor kopyayı kendisi alır
            frame = np.asanyarray(color_frame.get_data())
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # FIX ADIM-B: 640×480 → 320×240 (921KB → 230KB)
            # DDS serialize süresi 70ms → ~15ms, rs_cap 10fps → 30fps hedefi.
            # INTER_NEAREST: en hızlı — kalite fark etmez, sadece GUI preview.
            return cv2.resize(rgb, (320, 240), interpolation=cv2.INTER_NEAREST)

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
        # FIX SORUN-1: ZED capture ve publish ayri thread'lerde kosacak.
        # Event: capture thread frame'i holder'a koydugunda publish thread'i uyandirır.
        self._zed_pub_event = threading.Event()
        self._pc_interval_ns = int(1e9 / 5)
        self._next_pc_capture_ns = 0

        # ZED başlat + capture thread + publish thread (FIX SORUN-1: ayri thread'ler)
        self.camera = None
        self._zed_thread = None
        self._zed_pub_thread = None
        if CAMERA_AVAILABLE:
            try:
                self.camera = cam(self.info)
                self.get_logger().info('ZED kamera baslatildi.')
                self._zed_thread = threading.Thread(
                    target=self._zed_capture_loop, daemon=True, name='zed_capture')
                self._zed_thread.start()
                # FIX SORUN-1: publish ayri thread'de — grab() publish'i beklemez
                self._zed_pub_thread = threading.Thread(
                    target=self._zed_publish_loop, daemon=True, name='zed_publish')
                self._zed_pub_thread.start()
            except Exception as e:
                self.get_logger().error(f'ZED baslatılamadı: {str(e)}')
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

        # ZED ve RS artık kendi capture thread'lerinde doğrudan publish ediyor (timer YOK).
        # Sadece PC timer kalıyor — 5 Hz, ZED capture loop'tan bağımsız.
        self._zed_timer = None  # devre dışı — capture loop publish ediyor
        self._rs_timer  = None  # devre dışı — capture loop publish ediyor

        # FIX ADIM-E: ZED 640×360 → 320×180 (691KB → 173KB)
        # _zed_publish_loop resize+serialize toplamı 56-110ms → 33ms budget aşıyordu.
        # 4x küçük buffer: zed_cb 56-110ms → ~15ms hedefi, gui_zed 3fps → 25fps+.
        self._zed_msg = Image()
        self._zed_msg.encoding = 'rgb8'
        self._zed_msg.is_bigendian = False
        self._zed_msg.height = 180
        self._zed_msg.width  = 320
        self._zed_msg.step   = 320 * 3
        self._zed_buf = bytearray(320 * 180 * 3)
        self._zed_msg.data = self._zed_buf
        # Pre-allocated numpy view: np.frombuffer her frame'de oluşturulmuyor
        self._zed_np_view = np.frombuffer(self._zed_buf, dtype=np.uint8)
        # FIX ADIM-B: RealSense 640×480 → 320×240 (921,600 → 230,400 bytes)
        # DDS serialize baskısı 4x azalır: rs_cb 70ms → ~15ms hedefi.
        self._rs_msg = Image()
        self._rs_msg.encoding = 'rgb8'
        self._rs_msg.is_bigendian = False
        self._rs_msg.height = 240
        self._rs_msg.width  = 320
        self._rs_msg.step   = 320 * 3
        self._rs_buf = bytearray(320 * 240 * 3)
        self._rs_msg.data = self._rs_buf
        self._rs_np_view = np.frombuffer(self._rs_buf, dtype=np.uint8)
        # DEPTH_MODE.NONE: PC devre dışı — timer oluşturulmuyor
        self._pc_timer = None
        self._last_point_cloud = None
        self._pc_data_lock = threading.Lock()
        # ZED publish interval jitter ölçümü
        self._zed_last_pub_ns: int = 0
        self._zed_pub_interval_max_ms: float = 0.0
        self._zed_stutter_count: int = 0
        self._perf.add_value('zed_interval_max', lambda: f'{self._zed_pub_interval_max_ms:.1f}ms')
        self._perf.add_value('zed_stutter_cnt', lambda: str(self._zed_stutter_count))

    def _zed_capture_loop(self):
        """ZED capture loop — SADECE grab() + LatestFrameHolder.put() + event.set() yapar.
        FIX SORUN-1: Publish tamamen ayrı _zed_publish_loop thread'ine taşındı.
        grab() artık publish'i beklemez → GIL çakışması sıfır, jitter ortadan kalkar.
        """
        # FIX ADIM-D: Thread'e hafif OS öncelik avantajı — RS thread ve executor ile yarışı kazanır.
        # nice(-5): default 0'dan daha yüksek öncelik, root gerekmez (-20..19 arası).
        try:
            import os as _os
            _os.nice(-5)
        except OSError:
            pass  # root yoksa sessizce devam et
        # PROBE: sayaçlar
        _probe_zed_cap_count = 0
        _probe_zed_grab_sum = 0.0
        _probe_zed_put_sum  = 0.0
        _probe_zed_eset_sum = 0.0

        while self._running and self.camera is not None:
            try:
                # PROBE: grab süresi
                _t0 = time.monotonic()
                frame, _pc, frame_ns = self.camera.grab_frame()
                _grab_ms = (time.monotonic() - _t0) * 1000.0

                if frame is not None:
                    # ZED SDK gerçek capture timestamp (UNIX ns)
                    capture_ns = frame_ns if frame_ns is not None else time.time_ns()
                    self._zed_cap_fps.tick()

                    # PROBE: holder.put süresi
                    _t1 = time.monotonic()
                    self._zed_holder.put(frame, capture_ns=capture_ns)
                    _put_ms = (time.monotonic() - _t1) * 1000.0

                    # PROBE: event.set süresi
                    _t2 = time.monotonic()
                    self._zed_pub_event.set()
                    _eset_ms = (time.monotonic() - _t2) * 1000.0

                    # PROBE: per-frame print
                    print(f'[ZED] grab={_grab_ms:.2f}ms | holder_put={_put_ms:.3f}ms | event_set={_eset_ms:.3f}ms', flush=True)

                    # PROBE: SUMMARY her 30 frame'de bir
                    _probe_zed_grab_sum += _grab_ms
                    _probe_zed_put_sum  += _put_ms
                    _probe_zed_eset_sum += _eset_ms
                    _probe_zed_cap_count += 1
                    if _probe_zed_cap_count % 30 == 0:
                        _n = 30
                        print(
                            f'[SUMMARY] ZED_cap_fps≈{_n / (_probe_zed_grab_sum / 1000.0):.1f} '
                            f'| ZED_drop={self._zed_holder.dropped()} '
                            f'| grab_avg={_probe_zed_grab_sum/_n:.2f}ms '
                            f'| put_avg={_probe_zed_put_sum/_n:.3f}ms',
                            flush=True
                        )
                        _probe_zed_grab_sum = 0.0
                        _probe_zed_put_sum  = 0.0
                        _probe_zed_eset_sum = 0.0
            except Exception:
                time.sleep(0.001)  # hata döngüsünde CPU spike engelle

    def _zed_publish_loop(self):
        """ZED publish loop — _zed_pub_event ile uyandırılır, holder'dan frame alır, publish eder.
        FIX SORUN-1: grab() ile publish() GIL çakışması giderildi — tamamen ayrı thread'ler.
        grab() bir sonraki frame'e geçerken bu thread ROS serializasyonunu halleder.
        """
        # FIX ADIM-D: Publish thread'e de OS öncelik avantajı
        try:
            import os as _os
            _os.nice(-5)
        except OSError:
            pass
        # PROBE: sayaçlar
        _probe_pub_count   = 0
        _probe_wait_sum    = 0.0
        _probe_cvt_sum     = 0.0
        _probe_resize_sum  = 0.0
        _probe_copyto_sum  = 0.0
        _probe_pub_sum     = 0.0
        _probe_total_sum   = 0.0

        while self._running:
            # PROBE: event_wait süresi
            _tw0 = time.monotonic()
            signalled = self._zed_pub_event.wait(timeout=0.1)
            _wait_ms = (time.monotonic() - _tw0) * 1000.0

            if not signalled:
                continue
            self._zed_pub_event.clear()

            if not self._running:
                break

            try:
                zed_frame, _pc, capture_ns = self._zed_holder.get()
                if zed_frame is None:
                    continue

                capture_ns = capture_ns or time.time_ns()
                _t_total = time.monotonic()

                self._zed_cb_timer.start()
                start_pub_ns = time.monotonic_ns()

                # PROBE: cvtColor süresi
                _t0 = time.monotonic()
                zed_rgb = cv2.cvtColor(zed_frame, cv2.COLOR_RGBA2RGB)
                _cvt_ms = (time.monotonic() - _t0) * 1000.0

                # PROBE: resize süresi
                _t0 = time.monotonic()
                # FIX ADIM-E: 320×180 (173KB) — 691KB'den 4x küçük
                small = cv2.resize(zed_rgb, (320, 180), interpolation=cv2.INTER_NEAREST)
                _resize_ms = (time.monotonic() - _t0) * 1000.0

                # Pre-allocated buffer kontrolü
                needed = small.nbytes
                if len(self._zed_buf) != needed:
                    self._zed_buf = bytearray(needed)
                    self._zed_np_view = np.frombuffer(self._zed_buf, dtype=np.uint8)
                    self._zed_msg.height = small.shape[0]
                    self._zed_msg.width  = small.shape[1]
                    self._zed_msg.step   = small.shape[1] * small.shape[2]

                # PROBE: copyto süresi
                _t0 = time.monotonic()
                np.copyto(self._zed_np_view, small.ravel())
                _copyto_ms = (time.monotonic() - _t0) * 1000.0

                self._zed_msg.data = self._zed_buf
                self._zed_msg.header.stamp.sec = capture_ns // 1_000_000_000
                self._zed_msg.header.stamp.nanosec = capture_ns % 1_000_000_000
                self._zed_msg.header.frame_id = 'zed_camera_link'

                # PROBE: publish süresi
                _t0 = time.monotonic()
                self.zed_publisher.publish(self._zed_msg)
                _pub_ms = (time.monotonic() - _t0) * 1000.0

                _total_ms = (time.monotonic() - _t_total) * 1000.0
                self._zed_pub_latency_ms = (time.monotonic_ns() - start_pub_ns) / 1e6

                # PROBE: per-frame print
                print(
                    f'[ZED_PUB] event_wait={_wait_ms:.2f}ms | cvtColor={_cvt_ms:.2f}ms | '
                    f'resize={_resize_ms:.2f}ms | copyto={_copyto_ms:.3f}ms | '
                    f'publish={_pub_ms:.2f}ms | TOTAL={_total_ms:.2f}ms',
                    flush=True
                )

                # PROBE: SUMMARY her 30 frame'de bir
                _probe_wait_sum   += _wait_ms
                _probe_cvt_sum    += _cvt_ms
                _probe_resize_sum += _resize_ms
                _probe_copyto_sum += _copyto_ms
                _probe_pub_sum    += _pub_ms
                _probe_total_sum  += _total_ms
                _probe_pub_count  += 1
                if _probe_pub_count % 30 == 0:
                    _n = 30
                    print(
                        f'[SUMMARY] ZED_pub_fps≈{1000.0*_n/_probe_total_sum:.1f} '
                        f'| wait_avg={_probe_wait_sum/_n:.2f}ms '
                        f'| cvt_avg={_probe_cvt_sum/_n:.2f}ms '
                        f'| resize_avg={_probe_resize_sum/_n:.2f}ms '
                        f'| copyto_avg={_probe_copyto_sum/_n:.3f}ms '
                        f'| pub_avg={_probe_pub_sum/_n:.2f}ms '
                        f'| total_avg={_probe_total_sum/_n:.2f}ms',
                        flush=True
                    )
                    _probe_wait_sum   = 0.0
                    _probe_cvt_sum    = 0.0
                    _probe_resize_sum = 0.0
                    _probe_copyto_sum = 0.0
                    _probe_pub_sum    = 0.0
                    _probe_total_sum  = 0.0

                # Publish interval jitter ölçümü
                if self._zed_last_pub_ns > 0:
                    interval_ms = (capture_ns - self._zed_last_pub_ns) / 1e6
                    if interval_ms > self._zed_pub_interval_max_ms:
                        self._zed_pub_interval_max_ms = interval_ms
                    if interval_ms > 50.0:
                        self._zed_stutter_count += 1
                        self.get_logger().warn(
                            f'[STUTTER] ZED publish interval {interval_ms:.1f}ms '
                            f'(toplam: {self._zed_stutter_count})')
                self._zed_last_pub_ns = capture_ns
                self._perf.tick()
            except Exception as pub_err:
                self.get_logger().error(f'ZED publish hatası: {pub_err}')
            finally:
                self._zed_cb_timer.stop()

    def _rs_capture_loop(self):
        """RealSense capture loop — kendi thread'inde sürekli çalışır ve doğrudan publish eder.
        Timer'a bırakmak yerine capture anında publish: quantization delay ortadan kalkar.
        """
        # PROBE: sayaçlar
        _probe_rs_count      = 0
        _probe_rs_getf_sum   = 0.0
        _probe_rs_copyto_sum = 0.0
        _probe_rs_pub_sum    = 0.0
        _probe_rs_total_sum  = 0.0

        while self._running and self.realsense is not None:
            try:
                # PROBE: get_frame süresi
                _t0 = time.monotonic()
                frame = self.realsense.get_frame()
                _getf_ms = (time.monotonic() - _t0) * 1000.0

                if frame is not None:
                    capture_ns = time.time_ns()
                    self._rs_cap_fps.tick()
                    self._rs_holder.put(frame, capture_ns=capture_ns)
                    _t_total = time.monotonic()
                    try:
                        self._rs_cb_timer.start()
                        start_pub_ns = time.monotonic_ns()
                        needed = frame.nbytes
                        if len(self._rs_buf) != needed:
                            self._rs_buf = bytearray(needed)
                            self._rs_np_view = np.frombuffer(self._rs_buf, dtype=np.uint8)
                            self._rs_msg.height = frame.shape[0]
                            self._rs_msg.width  = frame.shape[1]
                            self._rs_msg.step   = frame.shape[1] * frame.shape[2]

                        # PROBE: copyto süresi
                        _t1 = time.monotonic()
                        np.copyto(self._rs_np_view, frame.ravel())
                        _copyto_ms = (time.monotonic() - _t1) * 1000.0

                        self._rs_msg.data = self._rs_buf
                        self._rs_msg.header.stamp = self.get_clock().now().to_msg()
                        self._rs_msg.header.frame_id = 'realsense_camera_link'

                        # PROBE: publish süresi
                        _t1 = time.monotonic()
                        self.realsense_publisher.publish(self._rs_msg)
                        _pub_ms = (time.monotonic() - _t1) * 1000.0

                        _total_ms = (time.monotonic() - _t_total) * 1000.0
                        self._rs_pub_latency_ms = (time.monotonic_ns() - start_pub_ns) / 1e6

                        # PROBE: per-frame print
                        print(
                            f'[RS] get_frame={_getf_ms:.2f}ms | copyto={_copyto_ms:.3f}ms | '
                            f'publish={_pub_ms:.2f}ms | TOTAL={_total_ms:.2f}ms',
                            flush=True
                        )

                        # PROBE: SUMMARY her 30 frame'de bir
                        _probe_rs_getf_sum   += _getf_ms
                        _probe_rs_copyto_sum += _copyto_ms
                        _probe_rs_pub_sum    += _pub_ms
                        _probe_rs_total_sum  += _total_ms
                        _probe_rs_count      += 1
                        if _probe_rs_count % 30 == 0:
                            _n = 30
                            print(
                                f'[SUMMARY] RS_fps≈{1000.0*_n/_probe_rs_getf_sum:.1f} '
                                f'| RS_drop={self._rs_holder.dropped()} '
                                f'| getframe_avg={_probe_rs_getf_sum/_n:.2f}ms '
                                f'| copyto_avg={_probe_rs_copyto_sum/_n:.3f}ms '
                                f'| pub_avg={_probe_rs_pub_sum/_n:.2f}ms '
                                f'| total_avg={_probe_rs_total_sum/_n:.2f}ms',
                                flush=True
                            )
                            _probe_rs_getf_sum   = 0.0
                            _probe_rs_copyto_sum = 0.0
                            _probe_rs_pub_sum    = 0.0
                            _probe_rs_total_sum  = 0.0

                        self._perf.tick()
                    except Exception as pub_err:
                        self.get_logger().error(f'RS inline publish hatası: {pub_err}')
                    finally:
                        self._rs_cb_timer.stop()
                else:
                    # FIX SORUN-4: None gelince CPU spin engelle
                    time.sleep(0.001)
            except Exception:
                time.sleep(0.001)  # hata döngüsünde CPU spike engelle


    def destroy_node(self):
        """Node kapanırken thread'leri durdur."""
        self._running = False
        self._zed_pub_event.set()
        for t in (self._zed_thread, self._zed_pub_thread, self._rs_thread):
            if t and t.is_alive():
                t.join(timeout=1.0)
        if self._zed_timer is not None:
            self._zed_timer.cancel()
        if self._rs_timer is not None:
            self._rs_timer.cancel()
        # pc_timer: DEPTH_MODE.NONE iken None — kontrol et
        if self._pc_timer is not None:
            self._pc_timer.cancel()
        super().destroy_node()
    

def main(args=None):
    # ── GC Optimizasyonu ──
    # gc.freeze(): import sırasında oluşturulan nesneleri ölümsüz nesle taşı → gen-2 tarama süresi azalır.
    gc.freeze()
    gc.set_threshold(1000, 15, 15)

    rclpy.init(args=args)
    camera_node = CameraNode()
    # FIX ADIM-A: MultiThreadedExecutor(4) → SingleThreadedExecutor
    # camera_node'unda ROS callback YOK (ZED/RS timer devre dışı).
    # 4 boş executor thread GIL için yarışıyor → _zed_publish_loop event.wait()
    # içinde 133–267ms beklemek zorunda kalıyordu (thread starvation).
    # SingleThreadedExecutor: GIL baskısı sıfır, publish starvation ortadan kalkar.
    executor = SingleThreadedExecutor()
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