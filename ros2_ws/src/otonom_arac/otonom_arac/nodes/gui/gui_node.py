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

import cv2
# cv2 import sonrası plugin yolunu tekrar doğrula (cv2 override edebilir).
if os.environ.get('QT_QPA_PLATFORM_PLUGIN_PATH', '') not in _QT_CANDIDATES:
    for _candidate in _QT_CANDIDATES:
        if os.path.isdir(_candidate):
            os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = _candidate
            break
import numpy as np
import threading

import sys
try:
    import pyqtgraph as pg
    from sklearn.cluster import DBSCAN
    ADVANCED_LIDAR = True
except ImportError:
    ADVANCED_LIDAR = False
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel,
    QGridLayout, QVBoxLayout, QHBoxLayout,
    QPushButton, QTextEdit
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap


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
            color_map = [
                (255, 0, 0), (0, 255, 0), (0, 0, 255),
                (255, 255, 0), (255, 0, 255), (0, 255, 255)
            ]
            spots = []
            for idx, (x, y) in enumerate(points):
                label = labels[idx]
                if label == -1:
                    color = (100, 100, 100)
                else:
                    cname = label_to_name.get(label, '?')
                    color = color_map[ord(cname) % len(color_map)]
                spots.append({'pos': (x, y), 'brush': pg.mkBrush(*color)})
            self.scatter.setData(spots)

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
            self._dbscan = DBSCAN(eps=200, min_samples=4)
        self._lidar_raw = None          # SORUN 7: ham lidar frame'i background worker'a taşımak için
        self._lidar_raw_lock = threading.Lock()
        self._dbscan_busy = False
        self._dbscan_spawn_lock = threading.Lock()  # double-check guard: tek thread spawn garantisi

        # Bird Eye ve nesne tespiti
        self.bev_frame = None
        self.detection_text = "Nesne yok"

        # SORUN 4: Frame değişkenleri (zed/realsense/bev) iki thread'den erişiliyor;
        # ROS spin thread yazar, Qt main thread okur — lock zorunlu.
        self._frame_lock = threading.Lock()

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
        self.create_subscription(LaserScan, '/lidar/scan', self.lidar_callback, 10)

    # ─── Callback'ler ───────────────────────────────────
    def zed_callback(self, msg):
        frame = self.imgmsg_to_cv2(msg)
        with self._frame_lock:
            self.zed_frame = frame

    def realsense_callback(self, msg):
        frame = self.imgmsg_to_cv2(msg)
        with self._frame_lock:
            self.realsense_frame = frame

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

    def detection_callback(self, msg):
        self.detection_text = msg.data

    def lidar_callback(self, msg):
        if not ADVANCED_LIDAR:
            with self._lidar_lock:
                self.lidar_data = msg.ranges
            return

        # SORUN 7: Ham veriyi sakla, DBSCAN'i background thread'de calistir.
        # Boylece ROS spin thread aninda doner ve image callback'leri beklemez.
        with self._lidar_raw_lock:
            self._lidar_raw = msg
        if not self._dbscan_busy:
            with self._dbscan_spawn_lock:
                if not self._dbscan_busy:  # double-check: iki hızlı callback tek thread başlatır
                    threading.Thread(target=self._dbscan_worker, daemon=True).start()

    def _dbscan_worker(self):
        self._dbscan_busy = True
        try:
            with self._lidar_raw_lock:
                msg = self._lidar_raw
            if msg is None:
                return
            points = []
            for i, dist in enumerate(msg.ranges):
                if not (msg.range_min <= dist <= msg.range_max):
                    continue
                angle = msg.angle_min + i * msg.angle_increment
                x = dist * 1000 * np.sin(angle)
                y = dist * 1000 * np.cos(angle)
                points.append((x, y))
            if len(points) < 4:
                return
            points_np = np.array(points)
            labels = self._dbscan.fit(points_np).labels_
            label_to_name = self._cluster_tracker.update_clusters(points_np, labels)
            with self._lidar_lock:
                self.lidar_data = (points_np, labels, label_to_name)
        finally:
            self._dbscan_busy = False

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

        self.lbl_mod.setText(f"Mod: {node.mode_str}")
        self.lbl_hiz.setText(f"Hız: {node.ileri_geri}")
        self.lbl_direksiyon.setText(f"Direksiyon: {node.sag_sol}")
        self.lbl_vites.setText(f"Vites: {node.vites}")
        self.lbl_algoritma.setText(f"Algoritma: {node.algorithm}")

        with node._frame_lock:
            zed_snap = node.zed_frame
            rs_snap  = node.realsense_frame
            bev_snap = node.bev_frame

        if zed_snap is not None:
            self._show_frame(self.label_zed, zed_snap)
        if rs_snap is not None:
            self._show_frame(self.label_realsense, rs_snap)
        if bev_snap is not None:
            self._show_frame(self.label_birdeye, bev_snap)

        self.label_traffic.setText(f"Traffic Signage\n{node.detection_text}")

        with node._lidar_lock:
            lidar_snapshot = node.lidar_data
        if lidar_snapshot is not None:
            if ADVANCED_LIDAR:
                self.label_lidar.update_points(*lidar_snapshot)
            else:
                self.label_lidar.setText(f"Lidar\n{len(lidar_snapshot)} nokta")

    def _show_frame(self, label, frame):
        # frame zaten RGB, dönüşüm gereksiz
        h, w, ch = frame.shape
        qt_img = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_img)
        label.setPixmap(pixmap.scaled(label.width(), label.height(), Qt.KeepAspectRatio))


def main(args=None):
    rclpy.init(args=args)
    ros_node = GuiNode()

    # ROS2 spin ayrı thread'de — UI'yı bloke etmez
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(ros_node),
        daemon=True
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
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
