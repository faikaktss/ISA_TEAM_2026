from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QIcon
from lidar import LidarPlotWidget


class ClickableLabel(QLabel):
    clicked = pyqtSignal()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit()

class Ui_Form(object):
    def update_lidar(self,points,labels,label_to_name):
        if hasattr(self,"label_lidar"):
            self.label_lidar.update_points(points,labels,label_to_name)
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1250, 800)  # Form boyutunu artırdık
        Form.setStyleSheet("background-color: rgb(94, 94, 87);")

        Form.setWindowIcon(QIcon(r"C:\Users\byznr\Desktop\17.07.2025\logo.jpg"))


        # Orijinal görüntü alanı
        self.label_original = ClickableLabel(Form)
        self.label_original.setGeometry(QtCore.QRect(30, 10, 610, 400))  # Büyütülmüş boyut
        self.label_original.setObjectName("label_original")
 
        # Lidar görüntü alanı
        self.label_lidar = LidarPlotWidget(Form)
        self.label_lidar.setGeometry(QtCore.QRect(650, 20, 550, 370))  # Büyütülmüş boyut
        self.label_lidar.show()

        # Kenar görüntü alanı
        self.label_tSignage = ClickableLabel(Form)
        self.label_tSignage.setGeometry(QtCore.QRect(15, 400, 490, 350))  # Büyütülmüş boyut
        self.label_tSignage.setObjectName("label_tSignage")

        # Kuş bakışı görüntü alanı
        self.label_birdeye = ClickableLabel(Form)
        self.label_birdeye.setGeometry(QtCore.QRect(650, 400, 550, 330))  # Büyütülmüş boyut
        self.label_birdeye.setObjectName("label_birdeye")

        # Oynatma butonu
        self.pushButton_exit = QtWidgets.QPushButton(Form)
        self.pushButton_exit.setGeometry(QtCore.QRect(600, 750, 100, 40))  # Buton boyutunu büyüttük
        self.pushButton_exit.setStyleSheet("background-color: rgb(255, 228, 117); font-size: 14pt;")
        self.pushButton_exit.setObjectName("pushButton_exit")

        self.pushButton_info = QtWidgets.QPushButton(Form)
        self.pushButton_info.setGeometry(QtCore.QRect(840, 750, 100, 40))
        self.pushButton_info.setStyleSheet("background-color: rgb(255, 228, 117); font-size: 14pt;")
        self.pushButton_info.setObjectName("pushButton_info")
        self.pushButton_info.setText("Info")

        self.pushButton_Tbar = QtWidgets.QPushButton(Form)
        self.pushButton_Tbar.setGeometry(QtCore.QRect(400, 750, 100, 40))
        self.pushButton_Tbar.setStyleSheet("background-color: rgb(255, 228, 117); font-size: 14pt;")
        self.pushButton_Tbar.setObjectName("pushButton_Tbar")
        self.pushButton_Tbar.setText("Trackbar")

        self.pushButton_console = QtWidgets.QPushButton(Form)
        self.pushButton_console.setGeometry(QtCore.QRect(1060, 750, 100, 40))  # Konum ve boyut
        self.pushButton_console.setStyleSheet("background-color: rgb(255, 228, 117); font-size: 14pt;")
        self.pushButton_console.setObjectName("pushButton_console")
        self.pushButton_console.setText("Console")



        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

        # Başlık - Original
        self.label_title_original = QLabel(Form)
        self.label_title_original.setGeometry(QtCore.QRect(19, 10, 150, 30))  # Boyutu artırdım
        self.label_title_original.setStyleSheet("color: white; font-size: 18px; font-weight: bold;")  # Büyük ve kalın font
        self.label_title_original.setText("Original")

        # Başlık - Lidar
        self.label_title_lidar = QLabel(Form)
        self.label_title_lidar.setGeometry(QtCore.QRect(700, 20, 150, 30))  # Boyutu artırdım
        self.label_title_lidar.setStyleSheet("color: white; font-size: 18px; font-weight: bold;")  # Büyük ve kalın font
        #self.label_title_lidar.setText("Lidar")

        # Başlık - Edges
        self.label_title_tSignage = QLabel(Form)
        self.label_title_tSignage.setGeometry(QtCore.QRect(15, 400, 150, 30))  # Boyutu artırdım
        self.label_title_tSignage.setStyleSheet("color: white; font-size: 18px; font-weight: bold;")  # Büyük ve kalın font
        self.label_title_tSignage.setText("Traffic Signage")

        # Başlık - Bird Eye
        self.label_title_birdeye = QLabel(Form)
        self.label_title_birdeye.setGeometry(QtCore.QRect(650, 400, 150, 30))  # Boyutu artırdım
        self.label_title_birdeye.setStyleSheet("color: white; font-size: 18px; font-weight: bold;")  # Büyük ve kalın font
        self.label_title_birdeye.setText("Bird Eye")

    

        
    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "ISA-REVO ROBOTEAM  i.i."))
        self.pushButton_exit.setText(_translate("Form", "Exit"))
