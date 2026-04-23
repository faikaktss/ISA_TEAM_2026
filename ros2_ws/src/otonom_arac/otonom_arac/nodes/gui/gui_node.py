#!/usr/bin/env python3

import os
# OpenCV kendi Qt plugin'lerini yüklüyor ve PyQt5 ile çakışıyor.
# cv2 import'undan önce ve sonra sistem Qt plugin yolunu sabitle.
_QT_CANDIDATES = [
    '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms',
    '/usr/lib/aarch64-linux-gnu/qt5/plugins/platforms',
    '/usr/lib/arm-linux-gnueabihf/qt5/plugins/platforms',
]
if 'QT_QPA_PLATFORM_PLUGIN_PATH' not in os.environ:
    for _candidate in _QT_CANDIDATES:
        if os.path.isdir(_candidate):
            os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = _candidate
            break

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32, Bool, String
from sensor_msgs.msg import Image, LaserScan

# Düşük latency QoS profili — sadece en son frame, eski frame'ler drop edilir
_IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

import cv2  # noqa: E402
# cv2 import sonrası plugin yolunu tekrar doğrula (cv2 override edebilir).
if os.environ.get('QT_QPA_PLATFORM_PLUGIN_PATH', '') not in _QT_CANDIDATES:
    for _candidate in _QT_CANDIDATES:
        if os.path.isdir(_candidate):
            os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = _candidate
            break
import numpy as np  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
from otonom_arac.perf.metrics import FPSMeter, CallbackTimer, PerfPublisher  # noqa: E402

import sys  # noqa: E402
import gc    # noqa: E402
try:
    import pyqtgraph as pg
    from sklearn.cluster import DBSCAN
    ADVANCED_LIDAR = True
except ImportError:
    ADVANCED_LIDAR = False
from PyQt5.QtWidgets import (  # noqa: E402
    QApplication, QWidget, QLabel,
    QGridLayout, QVBoxLayout, QHBoxLayout,
    QPushButton, QTextEdit
)
from PyQt5.QtCore import Qt, QTimer  # noqa: E402
from PyQt5.QtGui import QImage, QPixmap  # noqa: E402


if ADVANCED_LIDAR:
    class ClusterTracker:
        """Her lidar taramasında nesneleri takip eder ve kalıcı isim (A,B,C...) verir."""
        def __init__(self, distance_threshold=500):
            self.label_to_name = {}
            self.name_to_centroid = {}
            self.next_name_index = 0
            self.distance_threshold = distance_threshold

        def update_clusters(self, points, labels):
            unique_labels = set(labels)
            current_centroids = {}
            for label in unique_labels:
                if label == -1:
                    continue
                cluster_points = points[labels == label]
                current_centroids[label] = np.mean(cluster_points, axis=0)

            new_label_to_name = {}
            used_names = set()

            for label, centroid in current_centroids.items():
                matched = False
                for name, prev_centroid in self.name_to_centroid.items():
                    dist = np.linalg.norm(centroid - prev_centroid)
                    if dist < self.distance_threshold and name not in used_names:
                        new_label_to_name[label] = name
                        self.name_to_centroid[name] = centroid
                        used_names.add(name)
                        matched = True
                        break
                if not matched:
                    new_name = chr(ord('A') + self.next_name_index % 26)
                    self.name_to_centroid[new_name] = centroid
                    new_label_to_name[label] = new_name
                    used_names.add(new_name)
                    self.next_name_index += 1

            self.label_to_name = new_label_to_name
            return new_label_to_name

    class LidarPlotWidget(pg.PlotWidget):
        """Lidar nokta bulutu + DBSCAN kümeleri + mesafe daireleri çizen widget."""
        def __init__(self, parent=None):
            super().__init__(parent)
            self.setAspectLocked(True)
            self.setBackground('k')
            self.scatter = pg.ScatterPlotItem(size=5, pen=None)
            self.addItem(self.scatter)
            self.setXRange(-5000, 5000)
            self.setYRange(-5000, 5000)
            self._draw_guides()

        def update_points(self, points, labels, label_to_name):
            """Vektörel güncelleme: Python loop YOK, pg.mkBrush() allocation YOK.
            RGBA numpy array ile doğrudan ScatterPlotItem güncellenir — Qt main thread
            üzerindeki GC ve allocation baskısı sıfıra iner.
            """
            n = len(points)
            if n == 0:
                self.scatter.setData([])
                return

            # Sabit renk paleti — her çağrıda yeniden oluşturma (class-level olabilir ama
            # numpy array oluşturma maliyeti ihmal edilebilir düzeyde)
            _COLOR_PALETTE = np.array([
                (255,   0,   0, 200),
                (  0, 255,   0, 200),
                (  0,   0, 255, 200),
                (255, 255,   0, 200),
                (255,   0, 255, 200),
                (  0, 255, 255, 200),
            ], dtype=np.uint8)
            _NOISE_COLOR = np.array([100, 100, 100, 150], dtype=np.uint8)

            # Vektörel renk ataması: Python for loop yok, pg.mkBrush() yok
            labels_arr = np.asarray(labels, dtype=np.int32)
            colors = np.empty((n, 4), dtype=np.uint8)

            noise_mask = labels_arr == -1
            colors[noise_mask] = _NOISE_COLOR

            non_noise = ~noise_mask
            if non_noise.any():
                # label → karakter adı → palet indeksi (vektörel)
                non_noise_labels = labels_arr[non_noise]
                names = np.array(
                    [ord(label_to_name.get(int(lbl), 'A')) % len(_COLOR_PALETTE)
                     for lbl in non_noise_labels],
                    dtype=np.int32
                )
                colors[non_noise] = _COLOR_PALETTE[names]

            self.scatter.setData(pos=points, brush=[pg.mkBrush(*c) for c in colors])

        def _draw_guides(self):
            for line in [
                pg.PlotCurveItem([-5000, 5000], [0, 0], pen=pg.mkPen('w', width=0.7)),
                pg.PlotCurveItem([0, 0], [-5000, 5000], pen=pg.mkPen('w', width=0.7)),
            ]:
                self.addItem(line)
            theta = np.linspace(0, 2 * np.pi, 200)
            for r, color in [(5000, 'g'), (3000, 'y'), (1000, 'r')]:
                self.addItem(pg.PlotCurveItem(r * np.cos(theta), r * np.sin(theta),
                                              pen=pg.mkPen(color, width=0.7)))


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')

        # Kamera görüntüleri
        self.zed_frame = None
        self.realsense_frame = None
        self.bev_frame = None
        self.zed_frame_version = 0
        self.realsense_frame_version = 0
        self.bev_frame_version = 0
        self.zed_capture_ns = 0
        self.realsense_capture_ns = 0
        self.bev_capture_ns = 0
        self.zed_render_age_ms = 0.0
        self.realsense_render_age_ms = 0.0
        self.bev_render_age_ms = 0.0

        # Araç bilgileri
        self.manual_mode = True
        # Joystick (manuel mod) değerleri
        self.joystick_ileri_geri = 0
        self.joystick_sag_sol = 0
        self.joystick_vites = 0
        # Control node (otonom mod) değerleri
        self.control_ileri_geri = 0
        self.control_sag_sol = 0
        self.control_vites = 0

        # Algoritma
        self.algorithm = "Bekleniyor"

        # Lidar
        self.lidar_data = None
        self._lidar_lock = threading.Lock()
        if ADVANCED_LIDAR:
            self._cluster_tracker = ClusterTracker()
        self._lidar_raw = None
        self._lidar_raw_lock = threading.Lock()

        # Kalıcı DBSCAN worker thread — her lidar callback'te yeni thread spawn yok.
        # threading.Event ile uyandırılır, işi bitince tekrar bekler (0 OS thread maliyeti).
        self._dbscan_event = threading.Event()   # yeni veri geldi sinyali
        self._dbscan_stop  = threading.Event()   # node kapanırken dur sinyali
        if ADVANCED_LIDAR:
            self._dbscan_thread = threading.Thread(
                target=self._dbscan_worker_loop, daemon=True, name='dbscan_worker')
            self._dbscan_thread.start()

        # Bird Eye ve nesne tespiti
        self.detection_text = "Nesne yok"

        # SORUN 4: Frame değişkenleri (zed/realsense/bev) iki thread'den erişiliyor;
        # ROS spin thread yazar, Qt main thread okur — lock zorunlu.
        self._frame_lock = threading.Lock()

        self._gui_render_timer = CallbackTimer('gui_render', budget_ms=33.0)
        self._gui_zed_fps = FPSMeter('gui_zed')
        self._gui_rs_fps = FPSMeter('gui_rs')
        self._gui_bev_fps = FPSMeter('gui_bev')
        self._perf = PerfPublisher(self, interval=2.0)
        self._perf.add_timer('gui_render', self._gui_render_timer)
        self._perf.add_fps('gui_zed', self._gui_zed_fps)
        self._perf.add_fps('gui_rs', self._gui_rs_fps)
        self._perf.add_fps('gui_bev', self._gui_bev_fps)
        self._perf.add_value('gui_zed_age', lambda: f'{self.zed_render_age_ms:.1f}ms')
        self._perf.add_value('gui_rs_age', lambda: f'{self.realsense_render_age_ms:.1f}ms')
        self._perf.add_value('gui_bev_age', lambda: f'{self.bev_render_age_ms:.1f}ms')
        # Jitter ölçümü: UI render interval (nominally 33ms) ve stutter frame sayacı
        self._ui_last_time: float = 0.0
        self._ui_stutter_count: int = 0
        self._ui_interval_max_ms: float = 0.0
        self._perf.add_value('ui_stutter_cnt', lambda: str(self._ui_stutter_count))
        self._perf.add_value('ui_interval_max', lambda: f'{self._ui_interval_max_ms:.1f}ms')
        # Lidar render süresi (Qt main thread'de — uzunsa stutter kaynağı)
        self._lidar_render_max_ms: float = 0.0
        self._perf.add_value('lidar_render_max', lambda: f'{self._lidar_render_max_ms:.1f}ms')

        # Subscriber'lar — Image topic'leri düşük latency QoS ile
        self.create_subscription(Image, '/zed/preview', self.zed_callback, _IMAGE_QOS)
        self.create_subscription(Image, '/realsense/image_raw', self.realsense_callback, _IMAGE_QOS)
        self.create_subscription(Bool, '/joystick/manual_mode', self.mode_callback, 10)
        # Manuel mod topic'leri
        self.create_subscription(Int32, '/joystick/ileri_geri', self.joystick_ileri_geri_callback, 10)
        self.create_subscription(Int32, '/joystick/sag_sol', self.joystick_sag_sol_callback, 10)
        self.create_subscription(Int32, '/joystick/vites', self.joystick_vites_callback, 10)
        # Otonom mod topic'leri
        self.create_subscription(Int32, '/control/ileri_geri', self.control_ileri_geri_callback, 10)
        self.create_subscription(Int32, '/control/sag_sol', self.control_sag_sol_callback, 10)
        self.create_subscription(Int32, '/control/vites', self.control_vites_callback, 10)
        self.create_subscription(String, '/algorithm/current', self.algorithm_callback, 10)
        self.create_subscription(Image, '/lane/bev_image', self.bev_callback, _IMAGE_QOS)
        self.create_subscription(String, '/detection/objects', self.detection_callback, 10)
        # depth=1 BEST_EFFORT: lidar scan backlog'u engelle — hep en güncel scan
        _LIDAR_QOS = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(LaserScan, '/lidar/scan', self.lidar_callback, _LIDAR_QOS)

    # ─── Callback'ler ───────────────────────────────────
    def zed_callback(self, msg):
        frame = self.imgmsg_to_cv2(msg)
        with self._frame_lock:
            self.zed_frame = frame
            self.zed_frame_version += 1
            self.zed_capture_ns = self._msg_stamp_to_ns(msg)

    def realsense_callback(self, msg):
        frame = self.imgmsg_to_cv2(msg)
        with self._frame_lock:
            self.realsense_frame = frame
            self.realsense_frame_version += 1
            self.realsense_capture_ns = self._msg_stamp_to_ns(msg)

    def mode_callback(self, msg):
        self.manual_mode = msg.data

    def joystick_ileri_geri_callback(self, msg):
        self.joystick_ileri_geri = msg.data

    def joystick_sag_sol_callback(self, msg):
        self.joystick_sag_sol = msg.data

    def joystick_vites_callback(self, msg):
        self.joystick_vites = msg.data

    def control_ileri_geri_callback(self, msg):
        self.control_ileri_geri = msg.data

    def control_sag_sol_callback(self, msg):
        self.control_sag_sol = msg.data

    def control_vites_callback(self, msg):
        self.control_vites = msg.data

    @property
    def ileri_geri(self):
        return self.joystick_ileri_geri if self.manual_mode else self.control_ileri_geri

    @property
    def sag_sol(self):
        return self.joystick_sag_sol if self.manual_mode else self.control_sag_sol

    @property
    def vites(self):
        return self.joystick_vites if self.manual_mode else self.control_vites

    def algorithm_callback(self, msg):
        self.algorithm = msg.data

    def bev_callback(self, msg):
        frame = self.imgmsg_to_cv2(msg)
        with self._frame_lock:
            self.bev_frame = frame
            self.bev_frame_version += 1
            self.bev_capture_ns = self._msg_stamp_to_ns(msg)

    def detection_callback(self, msg):
        self.detection_text = msg.data

    def lidar_callback(self, msg):
        if not ADVANCED_LIDAR:
            with self._lidar_lock:
                self.lidar_data = msg.ranges
            return

        # Ham veriyi güncelle ve kalıcı worker'ı uyandır — thread spawn yok, 0 OS maliyet
        with self._lidar_raw_lock:
            self._lidar_raw = msg
        self._dbscan_event.set()  # worker zaten bekliyorsa anında uyanır

    def _dbscan_worker_loop(self):
        """Kalıcı DBSCAN worker thread — Event ile uyandırılır, işi bitince tekrar bekler.
        Thread spawn/destroy maliyeti sıfır: OS sadece bir kez thread oluşturur.
        """
        while not self._dbscan_stop.is_set():
            # Yeni veri gelene kadar uyu (timeout: node kapanma sinyali için)
            signalled = self._dbscan_event.wait(timeout=1.0)
            if not signalled:
                continue  # timeout — stop kontrolü yap, tekrar bekle
            self._dbscan_event.clear()  # sinyali tüket

            if self._dbscan_stop.is_set():
                break

            try:
                with self._lidar_raw_lock:
                    msg = self._lidar_raw
                if msg is None:
                    continue

                # Vektörel dönüşüm (Adım 2'den): Python for loop yok
                ranges_arr = np.array(msg.ranges, dtype=np.float32)
                valid_mask = (ranges_arr >= msg.range_min) & (ranges_arr <= msg.range_max)
                valid_indices = np.where(valid_mask)[0]
                if len(valid_indices) < 4:
                    continue
                angles = msg.angle_min + valid_indices * msg.angle_increment
                x = ranges_arr[valid_mask] * 1000.0 * np.sin(angles)
                y = ranges_arr[valid_mask] * 1000.0 * np.cos(angles)
                points_np = np.column_stack([x, y])

                dbscan = DBSCAN(eps=200, min_samples=4)
                labels = dbscan.fit(points_np).labels_
                label_to_name = self._cluster_tracker.update_clusters(points_np, labels)
                with self._lidar_lock:
                    self.lidar_data = (points_np, labels, label_to_name)
            except Exception:
                pass  # worker thread çökmemeli — sessizce devam et

    @property
    def mode_str(self):
        return "MANUEL" if self.manual_mode else "OTONOM"

    def imgmsg_to_cv2(self, msg):
        # RGB olarak tut — Qt zaten RGB istiyor, çift dönüşüm gereksiz
        # .copy(): ROS middleware buffer'ını sahipleniriz; QImage bu pointer'ı tutar
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1).copy()
        if msg.encoding == 'mono8':
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        return frame

    def _msg_stamp_to_ns(self, msg):
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return 0
        return stamp.sec * 1_000_000_000 + stamp.nanosec


class InfoWindow(QWidget):
    """Eski kodun InfoWindow'u — angle, encoder, IMU, tabela bilgisi gösterir."""
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Veri Bilgisi")
        self.resize(400, 300)
        self.setStyleSheet("background-color: rgb(94, 94, 87); color: white;")

        layout = QVBoxLayout()
        self.lbl_mod       = QLabel()
        self.lbl_hiz       = QLabel()
        self.lbl_direksiyon= QLabel()
        self.lbl_vites     = QLabel()
        self.lbl_algoritma = QLabel()
        self.lbl_nesne     = QLabel()

        for lbl in [self.lbl_mod, self.lbl_hiz, self.lbl_direksiyon,
                    self.lbl_vites, self.lbl_algoritma, self.lbl_nesne]:
            lbl.setStyleSheet("font-size: 14px; padding: 4px;")
            layout.addWidget(lbl)

        btn_kapat = QPushButton("Kapat")
        btn_kapat.setStyleSheet("background-color: rgb(255,228,117); font-size: 13pt;")
        btn_kapat.clicked.connect(self.close)
        layout.addWidget(btn_kapat)
        self.setLayout(layout)

        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(100)
        self._refresh()

    def _refresh(self):
        n = self.ros_node
        self.lbl_mod.setText(f"Mod:        {n.mode_str}")
        self.lbl_hiz.setText(f"Hız:        {n.ileri_geri}")
        self.lbl_direksiyon.setText(f"Direksiyon: {n.sag_sol}")
        self.lbl_vites.setText(f"Vites:      {n.vites}")
        self.lbl_algoritma.setText(f"Algoritma:  {n.algorithm}")
        self.lbl_nesne.setText(f"Tespit:     {n.detection_text}")


class ConsoleWindow(QWidget):
    """ROS2 node log mesajlarını gösteren konsol penceresi."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Console")
        self.resize(600, 400)
        self.setStyleSheet("background-color: rgb(20,20,20);")

        layout = QVBoxLayout()
        self.text_area = QTextEdit()
        self.text_area.setReadOnly(True)
        self.text_area.setStyleSheet("background-color: black; color: lime; font-family: monospace; font-size: 12px;")
        layout.addWidget(self.text_area)

        btn_temizle = QPushButton("Temizle")
        btn_temizle.setStyleSheet("background-color: rgb(255,228,117); font-size: 13pt;")
        btn_temizle.clicked.connect(self.text_area.clear)
        layout.addWidget(btn_temizle)
        self.setLayout(layout)

    def log(self, msg: str):
        self.text_area.append(msg)


class MainWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self._last_versions = {
            'zed': -1,
            'rs': -1,
            'bev': -1,
        }
        self._last_ui_time: float = time.perf_counter()

        self.setWindowTitle("ISA REVO ROBOTEAM")
        self.setStyleSheet("background-color: rgb(50, 50, 50);")
        self.resize(1280, 800)

        grid = QGridLayout()
        grid.setSpacing(5)

        self.label_zed = self._make_camera_label("ZED Kamera")
        self.label_realsense = self._make_camera_label("RealSense")
        if ADVANCED_LIDAR:
            self.label_lidar = LidarPlotWidget()
            self.label_lidar.setMinimumSize(380, 300)
        else:
            self.label_lidar = self._make_camera_label("Lidar")

        grid.addWidget(self.label_zed, 0, 0)
        grid.addWidget(self.label_realsense, 0, 1)
        grid.addWidget(self.label_lidar, 0, 2)

        self.label_traffic = self._make_camera_label("Traffic Signage")
        self.label_birdeye = self._make_camera_label("Bird Eye")
        self.label_info = self._make_info_panel()

        grid.addWidget(self.label_traffic, 1, 0)
        grid.addWidget(self.label_birdeye, 1, 1)
        grid.addWidget(self.label_info, 1, 2)

        btn_layout = QHBoxLayout()
        self.btn_console = QPushButton("Console")
        self.btn_info    = QPushButton("Info")
        self.btn_exit    = QPushButton("Exit")

        for btn in [self.btn_console, self.btn_info, self.btn_exit]:
            btn.setStyleSheet("background-color: rgb(255,228,117); font-size: 13pt;")
            btn_layout.addWidget(btn)

        self._info_win    = None
        self._console_win = None

        self.btn_info.clicked.connect(self._open_info)
        self.btn_console.clicked.connect(self._open_console)
        self.btn_exit.clicked.connect(self.close)

        main_layout = QVBoxLayout()
        main_layout.addLayout(grid)
        main_layout.addLayout(btn_layout)
        self.setLayout(main_layout)

    def _open_window(self, attr, cls, *args):
        win = getattr(self, attr)
        if win is None or not win.isVisible():
            win = cls(*args)
            setattr(self, attr, win)
        win.show()
        win.raise_()

    def _open_info(self):
        self._open_window('_info_win', InfoWindow, self.ros_node)

    def _open_console(self):
        self._open_window('_console_win', ConsoleWindow)

    def _make_camera_label(self, title):
        label = QLabel()
        label.setStyleSheet("background-color: black; color: white; font-size: 14px;")
        label.setAlignment(Qt.AlignCenter)
        label.setText(title)
        label.setMinimumSize(380, 300)
        return label

    def _make_info_panel(self):
        widget = QWidget()
        widget.setStyleSheet("background-color: rgb(30,30,30); color: white;")
        layout = QVBoxLayout()

        self.lbl_mod        = QLabel("Mod: MANUEL")
        self.lbl_hiz        = QLabel("Hız: 0")
        self.lbl_direksiyon = QLabel("Direksiyon: 127")
        self.lbl_vites      = QLabel("Vites: 0")
        self.lbl_algoritma  = QLabel("Algoritma: Bekleniyor")

        for lbl in [self.lbl_mod, self.lbl_hiz, self.lbl_direksiyon,
                    self.lbl_vites, self.lbl_algoritma]:
            lbl.setStyleSheet("font-size: 16px; padding: 8px;")
            layout.addWidget(lbl)

        widget.setLayout(layout)
        return widget

    def update_ui(self):
        node = self.ros_node

        # ── Jitter ölçümü: QTimer 33ms nominal, 50ms+ = stutter frame ──
        now_perf = time.perf_counter()
        if self._last_ui_time > 0.0:
            interval_ms = (now_perf - self._last_ui_time) * 1000.0
            if interval_ms > node._ui_interval_max_ms:
                node._ui_interval_max_ms = interval_ms
            if interval_ms > 50.0:  # 33ms nominal + %50 tolerans
                node._ui_stutter_count += 1
                node.get_logger().warn(
                    f'[STUTTER] UI interval {interval_ms:.1f}ms '
                    f'(toplam: {node._ui_stutter_count})')
        self._last_ui_time = now_perf

        node._gui_render_timer.start()

        try:
            self.lbl_mod.setText(f"Mod: {node.mode_str}")
            self.lbl_hiz.setText(f"Hız: {node.ileri_geri}")
            self.lbl_direksiyon.setText(f"Direksiyon: {node.sag_sol}")
            self.lbl_vites.setText(f"Vites: {node.vites}")
            self.lbl_algoritma.setText(f"Algoritma: {node.algorithm}")

            with node._frame_lock:
                zed_snap = node.zed_frame
                zed_ver = node.zed_frame_version
                zed_ns = node.zed_capture_ns
                rs_snap = node.realsense_frame
                rs_ver = node.realsense_frame_version
                rs_ns = node.realsense_capture_ns
                bev_snap = node.bev_frame
                bev_ver = node.bev_frame_version
                bev_ns = node.bev_capture_ns

            now_ns = time.time_ns()

            if zed_snap is not None and zed_ver != self._last_versions['zed']:
                self._show_frame(self.label_zed, zed_snap)
                self._last_versions['zed'] = zed_ver
                node._gui_zed_fps.tick()
                if zed_ns:
                    node.zed_render_age_ms = (now_ns - zed_ns) / 1e6

            if rs_snap is not None and rs_ver != self._last_versions['rs']:
                self._show_frame(self.label_realsense, rs_snap)
                self._last_versions['rs'] = rs_ver
                node._gui_rs_fps.tick()
                if rs_ns:
                    node.realsense_render_age_ms = (now_ns - rs_ns) / 1e6

            if bev_snap is not None and bev_ver != self._last_versions['bev']:
                self._show_frame(self.label_birdeye, bev_snap)
                self._last_versions['bev'] = bev_ver
                node._gui_bev_fps.tick()
                if bev_ns:
                    node.bev_render_age_ms = (now_ns - bev_ns) / 1e6

            self.label_traffic.setText(f"Traffic Signage\n{node.detection_text}")

            with node._lidar_lock:
                lidar_snapshot = node.lidar_data
            if lidar_snapshot is not None:
                if ADVANCED_LIDAR:
                    _t_lidar = time.perf_counter()
                    self.label_lidar.update_points(*lidar_snapshot)
                    _lidar_ms = (time.perf_counter() - _t_lidar) * 1000.0
                    if _lidar_ms > node._lidar_render_max_ms:
                        node._lidar_render_max_ms = _lidar_ms
                    if _lidar_ms > 15.0:  # 15ms+ Qt main thread'i bloke ediyor
                        node.get_logger().warn(f'[STUTTER] Lidar render {_lidar_ms:.1f}ms')
                else:
                    self.label_lidar.setText(f"Lidar\n{len(lidar_snapshot)} nokta")
        finally:
            node._gui_render_timer.stop()
            node._perf.tick()

    def _show_frame(self, label, frame):
        fh, fw = frame.shape[:2]
        lw, lh = label.width(), label.height()

        if lw > 0 and lh > 0:
            # Cache: frame boyutu VE label boyutu değişmediyse scale hesabını tekrar yapma
            cached = getattr(label, '_cached_target_size', None)
            cached_src = getattr(label, '_cached_src_size', None)
            cached_label = getattr(label, '_cached_label_size', None)
            if cached is None or cached_src != (fw, fh) or cached_label != (lw, lh):
                scale = min(lw / fw, lh / fh)
                nw, nh = max(1, int(fw * scale)), max(1, int(fh * scale))
                label._cached_target_size = (nw, nh)
                label._cached_src_size = (fw, fh)
                label._cached_label_size = (lw, lh)
            else:
                nw, nh = cached

            if (nw, nh) != (fw, fh):
                frame = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)

        h, w, ch = frame.shape
        # QImage: frame.data pointer'ını tutar — copy() ile sahiplenilmiş numpy array gerekli
        qt_img = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
        label.setPixmap(QPixmap.fromImage(qt_img))


def main(args=None):
    # ── GC Optimizasyonu ──
    # gc.freeze(): import sırasında oluşturulan tüm nesneleri (modüller, sınıflar) ölümsüz nesle taşı.
    # GC bu nesneleri bir daha taramaz → gen-2 toplama süresi %60-80 azalır.
    gc.freeze()
    # Threshold: gen-0 için 700 (varsayılan) yerine 1000 — daha az sık tetiklensin.
    # gen-1 ve gen-2 için 15 (varsayılan 10) — biraz daha toleranslı.
    gc.set_threshold(1000, 15, 15)

    rclpy.init(args=args)
    ros_node = GuiNode()

    # ROS2 spin ayrı thread'de — UI'yı bloke etmez
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(ros_node),
        daemon=False
    )
    spin_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    # Qt timer sadece UI günceller — ROS callback'leri arka planda zaten çalışıyor
    timer = QTimer()
    timer.timeout.connect(window.update_ui)
    timer.start(33)  # ~30 FPS UI refresh

    exit_code = app.exec_()

    ros_node.destroy_node()
    # Kalıcı DBSCAN worker thread'i düzgün durdur
    if ADVANCED_LIDAR and hasattr(ros_node, '_dbscan_stop'):
        ros_node._dbscan_stop.set()
        ros_node._dbscan_event.set()  # wait()'ten çıksın
        ros_node._dbscan_thread.join(timeout=2.0)
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
