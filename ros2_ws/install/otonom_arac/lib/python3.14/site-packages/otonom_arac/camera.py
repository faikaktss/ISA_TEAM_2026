# from ultralytics import YOLO
# import pyzed.sl as sl
# import cv2
# import math
# from info import Info 

# class cam():
#     # nesneler oluşturuldu
#     def __init__(self,model_path,info: Info):
#         self.zed = sl.Camera()
#         self.image = sl.Mat()
#         self.point_cloud = sl.Mat()
#         self.depth = sl.Mat()
#         self.name = None
#         self.distance = None
        
#         self.model = YOLO(model_path)
#         self.info = info

#         init_params = sl.InitParameters()
#         init_params.camera_resolution = sl.RESOLUTION.HD1080
#         init_params.camera_fps = 30
#         init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
#         init_params.coordinate_units = sl.UNIT.CENTIMETER

#         self.zed.open(init_params)

#         cam_info = self.zed.get_camera_information()
#         resolution = cam_info.camera_configuration.resolution
#         self.width = resolution.width
#         self.height = resolution.height

#     # numpy array olarak img ve nokta bulutu döner
#     def img_and_point_cloud(self):
#         err = self.zed.grab()
#         if err == sl.ERROR_CODE.SUCCESS:
#             self.zed.retrieve_image(self.image, sl.VIEW.LEFT)  
#             image_numpy = self.image.get_data()  
#             self.img = cv2.cvtColor(image_numpy, cv2.COLOR_RGBA2RGB)
            
#             self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZ)  
#             return self.img, self.point_cloud
#         print("zed grab() hatası.", err)
#         exit()

#     # Nokta bulutundan mesafe hesaplama fonksiyonu
#     def get_distance(self, point_cloud, x, y):
#         err, point_cloud_value = point_cloud.get_value(x, y)
#         if err == sl.ERROR_CODE.SUCCESS:
#             x3d, y3d, z3d = point_cloud_value[0], point_cloud_value[1], point_cloud_value[2]
#             # Geçersiz nokta kontrolü
#             if x3d == 0 and y3d == 0 and z3d == 0:
#                 return float('inf')
#             return math.sqrt(x3d**2 + y3d**2 + z3d**2)
#         else:
#             return float('inf')

#     def object_detection(self, img, point_cloud):
#         results = self.model.predict(img,conf=0.5,device=0,imgsz=(736,736))
#         closest_object = None
#         min_distance = float('inf')
#         for r in results:
#             for box in r.boxes:
#                 cls = int(box.cls[0])
#                 conf = box.conf[0].item()
#                 if conf > 0.5:
#                     x, y, w, h = map(int, box.xywh[0])
#                     x1, y1, x2, y2 = map(int, box.xyxy[0])

#                     distance = self.get_distance(point_cloud, x, y)

#                     if distance < min_distance:
#                         min_distance = distance
#                         closest_object = (cls, round(conf, 2), round(distance, 2), x1, y1, x2, y2)

#         if closest_object:
#             cls, conf, distance, x1, y1, x2, y2 = closest_object
#             class_name = self.model.names[cls] if self.model.names and cls in self.model.names else str(cls)
#             label = f"{class_name} {distance:.2f}cm"
#             cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
#             cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)
#             self.name = cls
#             self.distance = distance

#             self.info.set_distance(distance)
#         else:
#             self.name = None
#             self.distance = None
#             self.info.set_distance(None)

#         return img

#Todo:ZED stereo kamerayı başlatıp hem renkli görüntüyü hem de 3 boyutlu derinlik haritasını
#Todo: anlık olarak yakalar.

import pyzed.sl as sl
import cv2
import math
from info import Info 

#Todo: ZED kamera ile görüntü yakalama ve nokta bulutu işleme sınıfı
class cam():
    def __init__(self, info: Info):
        self.zed = sl.Camera() #Todo: Fiziksel kamerayı temsil eder
        self.image = sl.Mat() # TOdo: Kameradan alınan görüntüyü tutar
        self.point_cloud = sl.Mat() # Todo: Her piksel için 3D koordinatları tutar
        self.depth = sl.Mat() # Todo: Derinlik haritası
        self.info = info

        #Todo: Kamera başlatma parametreleri
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = 30 # Todo: Kameranın saniyedeki kare sayısı
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.CENTIMETER

        #Todo: Kamerayı belirlediğimiz ayarlarla açar ve ışığı yakar
        self.zed.open(init_params)

        #Todo: Kameranın gerçek boyutlarını öğrenir
        cam_info = self.zed.get_camera_information()
        resolution = cam_info.camera_configuration.resolution
        self.width = resolution.width
        self.height = resolution.height

    def img_and_point_cloud(self):
        err = self.zed.grab() # Todo: Bana yeni bir kare ver
        if err == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)   # Todo: Burada sol taraftaki kamerayı alıyorum
            image_numpy = self.image.get_data()   # Todo: Görüntüyü numpy array formatına dönüştür
            self.img = cv2.cvtColor(image_numpy, cv2.COLOR_RGBA2RGB)
            
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZ)   # Todo: 3D nokta bulutunu al
            return self.img, self.point_cloud # Todo: Görüntüyü ve nokta bulutunu döndür
        print("zed grab() hatası.", err)
        exit()
    # Todo: Nokta bulutundan mesafe hesaplama fonksiyonu
    def get_distance(self, point_cloud, x, y):
        err, point_cloud_value = point_cloud.get_value(x, y)
        #Todo: Eğer nokta geçerliyse mesafeyi hesapla
        if err == sl.ERROR_CODE.SUCCESS:
            x3d, y3d, z3d = point_cloud_value[0], point_cloud_value[1], point_cloud_value[2]
            #Todo: Geçersiz nokta kontrolü
            if x3d == 0 and y3d == 0 and z3d == 0:
                return float('inf')
            #todo: Öklidyen mesafe formülü
            return math.sqrt(x3d**2 + y3d**2 + z3d**2)
        else:
            return float('inf')
