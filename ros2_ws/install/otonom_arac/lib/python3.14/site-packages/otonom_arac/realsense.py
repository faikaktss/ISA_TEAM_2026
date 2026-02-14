import pyrealsense2 as rs # Todo: Intel RealSense SDK
import numpy as np
import cv2
from ultralytics import YOLO

#Todo: Hem görüntü hem mesafe ölçümü yapar
class  RealSenseCam:
    #Todo: Kameranın renk ve derinlik verilerini alır, nesne tespiti yapar
    def __init__(self, model_path):
        #Todo: Kameradan gelen verilerin işlenmesi için gerekli ayarlar
        self.pipeline = rs.pipeline() # Todo: RealSense kamera akışını başlatır
        self.config = rs.config() # Todo: Kamera yapılandırma ayarlarını tutar
        self.model = YOLO(model_path) # Todo: Nesne tespiti için YOLO modeli yükler
        self.name = None # Todo: Tespit edilen nesnenin adı
        self.distance = None # Todo: Tespit edilen nesnenin mesafesi

        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30) # Todo: Derinlik akışını etkinleştirir
        self.config.enable_stream(rs.stream.color, 480, 270, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config) # Todo: Kamera akışını başlatır

        self.depth_sensor = self.profile.get_device().first_depth_sensor() # Todo: Derinlik sensörünü alır
        self.depth_scale = self.depth_sensor.get_depth_scale() #Todo: Derinlik ölçeğini alır

        align_to = rs.stream.color
        self.align = rs.align(align_to)

    #Todo:Derinlik  ve renk görüntüsünü hizalar
    def get_frame(self):
        frames = self.pipeline.wait_for_frames() #Todo: Kamera yeni bir kare üretinciye kadar bekle
        aligned_frames = self.align.process(frames) #Todo: Derinlik ve renk karelerini hizala
        depth = aligned_frames.get_depth_frame() #Todo: Derinlik karesini al
        color = aligned_frames.get_color_frame() #Todo: Renk karesini al

        if not depth or not color:
            return None, None, None

        return np.asanyarray(color.get_data()), depth, color


    # TODO: Nesne tespiti yapar
    def detect_objects(self):
        color_image, depth_frame, color_frame = self.get_frame()
        if color_image is None:
            return None

        # Todo: YOLO modeli ile nesne tespiti yap
        results = self.model(color_image, conf=0.5, imgsz=(736,736))[0]

        closest_object = None
        min_distance = float('inf')

        #Todo: Tespit edilen nesneler arasında en yakın olanı bul
        for box in results.boxes:   
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            distance = depth_frame.get_distance(cx, cy)

            if distance < min_distance:
                min_distance = distance
                closest_object = (cls, conf, distance, x1, y1, x2, y2)

        if closest_object:
            cls, conf, distance, x1, y1, x2, y2 = closest_object
            self.name = cls
            self.distance = round(distance * 100, 2)  # metre -> cm
            label = f"{self.model.names[cls]} {self.distance:.2f}cm"
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(color_image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            self.name = None
            self.distance = None

        return color_image

    def stop(self):
        self.pipeline.stop()
