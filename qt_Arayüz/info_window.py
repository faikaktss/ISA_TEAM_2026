from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout, QPushButton
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QIcon

class InfoWindow(QWidget):
    def __init__(self, info_obj):
        super().__init__()
        self.setWindowTitle("Data Information")
        self.resize(500, 400)
        self.setStyleSheet("background-color: rgb(94, 94, 87);")
        self.setWindowIcon(QIcon(r"C:\Users\byznr\Desktop\17.07.2025\logo.jpg"))

        self.info = info_obj

        self.layout = QVBoxLayout()

        self.label_angle = QLabel()
        self.label_line = QLabel()
        self.label_tabela = QLabel()
        self.label_encoder = QLabel()
        self.label_imu = QLabel()

        self.layout.addWidget(self.label_angle)
        self.layout.addWidget(self.label_line)
        self.layout.addWidget(self.label_tabela)
        self.layout.addWidget(self.label_encoder)
        self.layout.addWidget(self.label_imu)

        self.close_button = QPushButton("Kapat")
        self.close_button.setStyleSheet("background-color: rgb(255, 228, 117); font-size: 14pt;")
        self.close_button.clicked.connect(self.close)
        self.layout.addWidget(self.close_button)

        self.setLayout(self.layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_info)
        self.timer.start(100)

        self.update_info()

    def update_info(self):
        self.label_angle.setText(f"Açı (angle): {self.info.get_angle()}")
        self.label_line.setText(f"Lidar Engel Durumu: {self.info.get_line()}")
        self.label_tabela.setText(f"Algılanan Tabela: {self.info.get_tabela()}")
        self.label_encoder.setText(f"Encoder: {self.info.get_encoder()}")
        self.label_imu.setText(f"IMU: {self.info.get_imu()}")
