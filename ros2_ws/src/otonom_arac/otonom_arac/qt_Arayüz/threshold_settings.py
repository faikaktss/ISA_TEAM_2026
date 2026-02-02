from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QSpinBox, QPushButton
from PyQt5.QtGui import QIcon


class ThresholdSettingsWindow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Adaptive Threshold Settings")
        self.resize(300, 150)
        self.setStyleSheet("background-color: rgb(255, 228, 117);")

        self.layout = QVBoxLayout()

        self.label_blocksize = QLabel("Block Size:")
        self.spin_blocksize = QSpinBox()
        self.spin_blocksize.setRange(3, 255)
        self.spin_blocksize.setSingleStep(2)
        self.spin_blocksize.setValue(11)

        self.label_cval = QLabel("C Value:")
        self.spin_cval = QSpinBox()
        self.spin_cval.setRange(-100, 100)
        self.spin_cval.setValue(2)

        self.btn_apply = QPushButton("Apply")
        self.btn_apply.clicked.connect(self.apply_settings)

        self.layout.addWidget(self.label_blocksize)
        self.layout.addWidget(self.spin_blocksize)
        self.layout.addWidget(self.label_cval)
        self.layout.addWidget(self.spin_cval)
        self.layout.addWidget(self.btn_apply)

        self.setLayout(self.layout)

        self.block_size = 11
        self.c_value = 2

        self.result_window = None

    def apply_settings(self):
        block = self.spin_blocksize.value()
        if block % 2 == 0:
            block += 1
        self.block_size = block
        self.c_value = self.spin_cval.value()

        self.result_window = AppliedSettingsWindow(self.block_size, self.c_value, self)
        self.result_window.show()
        self.hide()


class AppliedSettingsWindow(QWidget):
    def __init__(self, block_size, c_value, settings_window):
        super().__init__()
        self.setWindowTitle("Applied Settings")
        self.resize(300, 150)
        self.setStyleSheet("background-color: rgb(94, 94, 87);")
        self.setWindowIcon(QIcon("qt_Arayüz/logo.jpg"))

        self.settings_window = settings_window

        self.layout = QVBoxLayout()

        self.label_block = QLabel(f"Block Size: {block_size}")
        self.label_cval = QLabel(f"C Value: {c_value}")
        self.label_block.setStyleSheet("font-size: 14pt;")
        self.label_cval.setStyleSheet("font-size: 14pt;")

        self.layout.addWidget(self.label_block)
        self.layout.addWidget(self.label_cval)

        self.btn_back = QPushButton("Go back and change the settings")
        self.btn_back.setStyleSheet("background-color: rgb(255, 228, 117); font-size: 12pt;")
        self.btn_back.clicked.connect(self.go_back)

        self.btn_close = QPushButton("Close")
        self.btn_close.setStyleSheet("background-color: rgb(255, 228, 117); font-size: 12pt;")
        self.btn_close.clicked.connect(self.close)

        self.layout.addWidget(self.btn_back)
        self.layout.addWidget(self.btn_close)

        self.setLayout(self.layout)

    def go_back(self):
        self.settings_window.show()
        self.close()
