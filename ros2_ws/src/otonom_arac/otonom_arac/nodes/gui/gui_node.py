#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String
from sensor_msgs.msg import Image

import cv2
import numpy as np

import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel,
    QGridLayout, QVBoxLayout, QHBoxLayout,
    QPushButton, QTextEdit
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QTextCursor
from PyQt5.QtCore import QPointF


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')

        # Kamera görüntüleri
        self.zed_frame = None
        self.realsense_frame = None

        # Araç bilgileri
        self.manual_mode = True
        self.ileri_geri = 0
        self.sag_sol = 127
        self.vites = 0

        # Algoritma
        self.algorithm = "Bekleniyor"

        # Lidar
        self.lidar_points = []

        # Subscriber'lar
        self.create_subscription(Image, '/zed/image_raw', self.zed_callback, 10)
        self.create_subscription(Image, '/realsense/image_raw', self.realsense_callback, 10)
        self.create_subscription(Bool, '/joystick/manual_mode', self.mode_callback, 10)
        self.create_subscription(Int32, '/joystick/ileri_geri', self.ileri_geri_callback, 10)
        self.create_subscription(Int32, '/joystick/sag_sol', self.sag_sol_callback, 10)
        self.create_subscription(Int32, '/joystick/vites', self.vites_callback, 10)
        self.create_subscription(String, '/algorithm/current', self.algorithm_callback, 10)

    # ─── Callback'ler ───────────────────────────────────
    def zed_callback(self, msg):
        self.zed_frame = self.imgmsg_to_cv2(msg)

    def realsense_callback(self, msg):
        self.realsense_frame = self.imgmsg_to_cv2(msg)

    def mode_callback(self, msg):
        self.manual_mode = msg.data

    def ileri_geri_callback(self, msg):
        self.ileri_geri = msg.data

    def sag_sol_callback(self, msg):
        self.sag_sol = msg.data

    def vites_callback(self, msg):
        self.vites = msg.data

    def algorithm_callback(self, msg):
        self.algorithm = msg.data

    def imgmsg_to_cv2(self, msg):
        """ROS2 Image mesajını OpenCV formatına çevirir"""
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)


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
        self.label_lidar = self._make_lidar_label("Lidar")

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

        self.btn_exit.clicked.connect(self.close)

        main_layout = QVBoxLayout()
        main_layout.addLayout(grid)
        main_layout.addLayout(btn_layout)
        self.setLayout(main_layout)

    # ─── Yardımcı fonksiyonlar ──────────────────────────
    def _make_camera_label(self, title):
        label = QLabel()
        label.setStyleSheet("background-color: black; color: white; font-size: 14px;")
        label.setAlignment(Qt.AlignCenter)
        label.setText(title)
        label.setMinimumSize(380, 300)
        return label

    def _make_lidar_label(self, title):
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

    def update(self):
        node = self.ros_node

        # Bilgi panelini güncelle
        mod_str = "MANUEL" if node.manual_mode else "OTONOM"
        self.lbl_mod.setText(f"Mod: {mod_str}")
        self.lbl_hiz.setText(f"Hız: {node.ileri_geri}")
        self.lbl_direksiyon.setText(f"Direksiyon: {node.sag_sol}")
        self.lbl_vites.setText(f"Vites: {node.vites}")
        self.lbl_algoritma.setText(f"Algoritma: {node.algorithm}")

        if node.zed_frame is not None:
            self._show_frame(self.label_zed, node.zed_frame)

        if node.realsense_frame is not None:
            self._show_frame(self.label_realsense, node.realsense_frame)

    def _show_frame(self, label, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qt_img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_img)
        label.setPixmap(pixmap.scaled(label.width(), label.height(), Qt.KeepAspectRatio))


def main(args=None):
    rclpy.init(args=args)
    ros_node = GuiNode()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    timer = QTimer()
    timer.timeout.connect(lambda: (
        rclpy.spin_once(ros_node, timeout_sec=0),
        window.update()
    ))
    timer.start(30)

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
