### bu kod Zed kullanmadan normal laptop kamerası ile kullanmak icin
### logitech kamera da baglanabilir 
### daha rahat yapı icin camera classi yapılabilir ve bu composition mantigi ile sadece tek main dosyasında kullanılabilir
### bu yapılırsa bu dosyaya gerek kalmaz


from LaneDetect import LaneDetection
#from camera import cam
from PyQt5.QtWidgets import QApplication
# from lidar import LidarSystem
from qt_Arayüz.inter2 import VideoApp
from ultralytics import YOLO
from utils import turnInfo,info_to_geojson_feature
from info import Info
#from arduino import Arduino,Teensy
#from alternatif2 import Command
import sys
import cv2

class_dict = {
        0: "kirmizi",
        1: "yesil",
        2: "park",
        3: "park_engelli",
        4: "sol",
        5: "sag",
        6: "sol",
        7: "sag_yasak",
        8: "sol_yasak",
        9: "dur",
        10: "durak",
        11: "sol",
        12: "ileri_sag",
        13: "sol",
        14: "sol",
        15: "sagdan_devam",
        16: "soldan_devam",
        17: "park_yasakiki",
        18: "yaya",
        19: "tunel",
        20: "ileri",
        21: "cift_yon"
    }


def main():
    app = QApplication(sys.argv)
    info = Info()
    window = VideoApp(info)
    model = YOLO("modeller/best.pt")
    lane_detection = LaneDetection()
    #zed = cam("best.pt",info=info)
    #elma = LidarSystem(port="COM6", enable_gui=True, print_clusters=True)
    #teensy = Teensy()
    #arduino = Arduino()
    #komut = Command(info,teensy,arduino,zed)

    # lidar_widget = elma.plot_widget
    # lidar_widget.setParent(window.ui.label_lidar)
    # lidar_widget.setGeometry(0,0,window.ui.label_lidar.width(),window.ui.label_lidar.height())
    # lidar_widget.show()
    # elma.start()
    
    cap = cv2.VideoCapture(0)
    window.show()
    tabela = []

    while window.isVisible():
        # frame, point_cloud = zed.img_and_point_cloud()
        ret,frame=cap.read()

        # if not point_cloud or frame is None:
        #     break

        # Şerit ve açı işlemleri
        station_result, angle= lane_detection.station_bird_eye_view(frame)
        result, edges = lane_detection.process(frame, 1920, 1080)
        bird, bird_canny, angle = lane_detection.bird_eye_view(frame)

        # Tabela tespiti + mesafe
        # signage_img = zed.object_detection(frame.copy(), point_cloud)

        # Görseli GUI'de göster
        window.process_frame(station_result, result, edges, bird, angle, None, signage_img=None)

        # original ve bird eye frameleri infoya aktarır
        info.set_originalFrame(frame)
        info.set_birdEyeFrame(bird)
        info.set_edgeBirdEye(bird_canny)

        # lane detection sonucunu alalım
        lane = lane_detection.lane_position
        info.set_lane(lane)


        # Bilgileri güncelle
        info.set_angle(angle)
        #arduino.setValue(info.get_angle())
        # info.get_engel(elma.engel)
        # komut.engel_komut()
        # info.set_encoder(arduino.encoder_distance())
        # info.set_imu(teensy.get_imu())
        # info.set_gps(gps_module.get_position())
        # info.set_tabela(class_dict.get(zed.name))
                            

        # # Eğer info penceresi açıksa GUI'yi güncelle
        # if window.info_window is not None:
        #     window.info_window.update_info()
            
        sol_red_count, sag_red_count = turnInfo(frame, 3, 3)


        # Tespit edilen tabelayı kaydet
        # tabela_adi = class_dict.get(zed.name)
        # tabela.append(tabela_adi)

        # En fazla 10 taneyi tut
    
        if len(tabela) > 10:
            tabela.pop(0)

        #  Mesafeyi konsola yazdır
        # if tabela_adi and zed.distance is not None:
        #     print(f"Tespit: {tabela_adi} | Mesafe: {zed.distance:.2f} cm")

        # Karar fonksiyonuna mesafeyi de gönder
        #komut.karar(tabela, sol_red_count, sag_red_count)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    sys.exit(app.exec_())
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()