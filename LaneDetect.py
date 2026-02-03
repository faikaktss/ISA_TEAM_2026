# Todo: Aracın ön kamerası sürekli görüntü veriyor. Biz bu görüntüden 
# Todo: sadece yolun olduğu bölgeyi kesiyoruz (ROI). Sonra o bölgede şerit çizgilerinin 
# Todo: kenarlarını çıkarıyoruz (threshold + Canny). Sonra bu kenar görüntüsünde doğru parçalarını 
# Todo: buluyoruz (HoughLinesP). Bu doğru parçalarını eğimlerine göre “sol şerit” ve “sağ şerit” diye ayırıyoruz. 
# Todo: Sol/sağ çizgilerin ortasını bulup aracın yol ortasına göre kaymasını (offset) ve direksiyon
# Todo: ihtiyacını (steering) hesaplıyoruz.


import cv2
import numpy as np
import math
from .error import Except
#Todo: Kameradan gelen görüntüden sadece yol kısmını alır ve şerit çizgilerinin kenarlarını ortaya çıkarır
class roi:
    def __init__(self,y1=400, y2=900, x1=500, x2=1400):
        self.y1=y1
        self.y2=y2
        self.x1=x1
        self.x2=x2
    #Todo: Roi boyutu gerçekten frame içine sığıyor mu Bunu kontrol eder. Kamera değil sadece yolu kontrol eder
    def get_roi(self,frame,width,height):
        if (self.x2 - self.x1 < width and self.y2 - self.y1 < height and self.x1 <= self.x2 and self.y2 >= self.y1):
            return frame[self.y1:self.y2 , self.x1:self.x2]
        else:
            e = Except("roi ayarlanamadı","get_roi()")
            print(e)
            exit()
    

    #Todo: Şerit çizgilerinin kenarlarını ortaya çıkarmaya yarar
    def edge_detection(self,frame):
        
        # Trackbar penceresini bir kez oluştur (ilk çalıştırmada)
        # if not hasattr(self, 'trackbar_initialized'):
        #     cv2.namedWindow("Threshold Control")
        #     cv2.createTrackbar("Block Size", "Threshold Control", 11, 50, nothing)
        #     cv2.createTrackbar("C Value", "Threshold Control", 2, 20, nothing)
        #     self.trackbar_initialized = True

        # # Trackbar değerlerini oku
        # block_size = cv2.getTrackbarPos("Block Size", "Threshold Control")
        # c_val = cv2.getTrackbarPos("C Value", "Threshold Control")

        # # blockSize tek sayı olmalı ve 3'ten küçük olmamalı
        # if block_size % 2 == 0:
        #     block_size += 1
        # if block_size < 3:
        #     block_size = 3

        # Griye çevir ve bulanıklaştır
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (25, 25), 0)


        # Todo: Şerit çizgileri beyaz asfalt siyah 
        mean_frame = cv2.mean(gray)[0]
        _,adaptive_thresh = cv2.threshold(
            blur,
            170 ,   #1.2*float(mean_frame),
            255,
            cv2.THRESH_BINARY
        )

        #Todo: siyah arka plan üzerinde beyaz çizgiler olur
        canny = cv2.Canny(adaptive_thresh, 50, 150)
        return canny
    
# Todo: Görüntüdeki değerin eğimini hesaplar
class angleCalculator:
    # Todo: Eğim bilgisi
    angle=None

    def find_slope(self,x1, y1, x2, y2):
        #Todo: Eğim i y'ye göre hesaplıyor
        if x2 - x1 != 0:
            return (x2- x1) / (y2 - y1)  
        else:
            print("Uyarı: Dikey doğru tespit edildi, eğim tanımsız.","find_slope()")
            return None  
    
    def slope_to_angle(self, slope):
        if slope is not None:
            #Todo: Eğimden açı hesaplanıyor
            angleCalculator.angle=-1*math.atan(slope) * 180 / math.pi # Todo: eğimi radyan cinsinden verir
            return angleCalculator.angle
        print("Açı hesaplanamadı.","slope_to_angle()")
        return None
    
class midpointFinder(angleCalculator):
    def __init__(self):
        super().__init__()
        self.kayma=0
        self.aci=0
    
    #Todo: Orta noktayı bulan fonksiyon
    def find_midpoint(self,line1,line2):
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        midpoint_x = (x1 + x2 + x3 + x4) // 4
        midpoint_y = (y1 + y2 + y3 + y4) // 4
        
        slope1 = self.find_slope(x1, y1, x2, y2)
        slope2 = self.find_slope(x3, y3, x4, y4)
       

        angle1 = self.slope_to_angle(slope1)
        angle2 = self.slope_to_angle(slope2)

        #print(f"Left angle of inclination: {angle1:.2f}, Right angle of inclination: {angle2:.2f}")
        return (midpoint_x, midpoint_y)

# Todo: Kuşbakışı görünüm dönüşümü
class birdEye:
    def __init__(self):
        #self.pts1 = np.float32([(192, 364), (101, 478), (502, 362),(561, 476)])
        self.pts1 = np.float32([(205, 329), (128,476), (504,330),(593,476)])
        #Todo: Hedef dörtgen koordinatları
        self.pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])

    def transform(self,frame):
        #Todo: Görüntüyü 640x480 boyutuna getirir
        frame =cv2.resize(frame,(640,480))
        #Todo: Perspektif dönüşüm matrisi hesaplanır
        matrix = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        #Todo: Perspektif dönüşümü uygulanır
        img = cv2.warpPerspective(frame, matrix, (640 ,480))
        #Todo: Görüntüyü kırpar
        imgTrim = img[:360,:]
        return img


#Todo: Özel duraklar için kuşbakışı görünüm dönüşümü
class stationBirdEye:
    def __init__(self):
       
        self.pts1 = np.float32([(406, 348), (404, 478), (596, 335), (619, 477)])
        self.pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])
        self.transformed = None
    def transform(self, frame):
        frame = cv2.resize(frame, (640, 480))
        matrix = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        self.transformed = transformed = cv2.warpPerspective(frame, matrix, (640, 480))
        return transformed

    
class LaneDetection:
    
    def __init__(self, lane_position="Unknow"):
        self.roi_object=roi() # Todo: Görüntünün sadece yol ile ilgili kısmını alır
        self.angle_calculator=angleCalculator() # Todo: Direksiyon açısını hesaplar
        self.midpoint_finder=midpointFinder() #Todo: Yolun sağ ve sol çizgileri arasındaki orta noktayı bulur
        self.bird_eye=birdEye() # Todo: Kuşbakışı görünüm dönüşümü
        self.lane_position=lane_position 
        self.kayma = None
        self.midPointVectorAngle=None
        self.kaymaEsik = 60 #bu sayi orta noktanin kaymasında esik olarak kullanılacak yani bu sayı kayma sayısından kucukse arac cok kaymis demektir.
        self.station_bird_eye = stationBirdEye()  

    def kaymaEsigi_ve_VirajKontrol(self):
        if(self.kayma is not None and self.kaymaEsik is not None):
            print("kaymahesaplaniyor")
            print(self.kaymaEsik < self.kaymaBuyukluk())
            #print(f"{self.midPointVectorAngle} ve yarma : {self.kayma}")

            return self.kaymaEsik <  self.kaymaBuyukluk()
        else:
            return False

            
    #Todo: Kaymanının büyüklüğünü döner
    def kaymaBuyukluk(self):
        return abs(self.kayma)
    

    #Todo:Kameradan gelen çiğ (ham) görüntüyü alır, içindeki gereksiz detayları çöpe atar ve sadece "Yol nerde
    #Todo ben nereye gitmeliyim?" sorusunun cevabını hesaplar.
    def process(self,frame,width,height):
            newroi=self.roi_object.get_roi(frame,width,height) #Todo: Görüntünün sadece yol ile ilgili kısmını alır
            edges=self.roi_object.edge_detection(newroi)#Todo: Şerit çizgilerinin kenarlarını ortaya çıkarır
            mask=edges.copy()#Todo: Maske oluşturur


            #Todo: Amaç gürültü temizleme
            pts = np.array([[186, 700], [453, 500], [940, 494], [1144, 760]], dtype=np.int32)
            pts = pts.reshape((-1,1,2))
            cv2.polylines(edges,[pts],True,(0,0,0),10)
            cv2.fillPoly(mask, [pts], (0, 0, 0))
            result = cv2.bitwise_and(edges, mask)

            #Todo: Hough Transform ile doğru parçalarını bulma Yani düz bir çizgi gibi görünen parçaları bulma
            #Todo: En az 50 piksel uzunluğundaki çizgileri alır
            lines=cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=50)
            result=np.copy(newroi)

            left_lines = []
            right_lines = []

            if lines is not None:
                #Todo: Bulunan çizgileri eğimlerine göre sol ve sağ olarak ayırma
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    slope = self.angle_calculator.find_slope(x1, y1, x2, y2)
                    #Todo: Eğim bilgisine göre sol ve sağ çizgileri ayırma
                    if slope is not None:
                        #Todo: Negatif eğim sol çizgi pozitif eğim sağ çizgi
                        if slope < 0:
                            left_lines.append((x1, y1, x2, y2))
                        #Todo: Pozitif eğim sağ çizgi
                        elif slope > 0:
                            right_lines.append((x1, y1, x2, y2))
                    cv2.line(result, (x1, y1), (x2, y2), (255, 0, 0), thickness=5)

            steering_angle = 0
            self.lane_position = "Unknown"

            if left_lines and right_lines:
                #Todo: Sol ve sağ çizgilerin ortalamasını alır
                left_avg = np.mean(left_lines, axis=0, dtype=int)
                right_avg = np.mean(right_lines, axis=0, dtype=int)

                #Todo: Ortalama çizgileri çiz
                midpoint_x, midpoint_y = self.midpoint_finder.find_midpoint(left_avg, right_avg)
                #Todo: Orta noktayı çiz
                x,y = edges.shape
                #Todo: Kayma ve açı hesaplama
                self.kayma = x-midpoint_x
                #Todo: Orta noktanın aracın merkezine göre açısını hesapla
                self.midPointVectorAngle = math.atan((x-midpoint_x)/(midpoint_y))*((180/math.pi))
                #Todo: Kayma ve açı bilgilerini yazdır
                print(f"{self.midPointVectorAngle} ve kayma : {self.kayma}")
                print(self.kayma)
                cv2.circle(result, (midpoint_x, midpoint_y), 10, (0, 255, 0), -1)
                cv2.imshow("orta",result)
                
                #Todo: Direksiyon açısını hesapla
                frame_center = (newroi.shape[1]) // 2
                lane_center = (left_avg[2] + right_avg[2]) // 2
                steering_angle = (lane_center - frame_center) * 0.1

                #Todo: Şerit pozisyonunu belirle
                if abs(self.kayma) < 10:
                    if lane_center < frame_center - 50:
                        self.lane_position = -1
                    elif lane_center > frame_center + 50:
                        self.lane_position = 1
                    else:
                        self.lane_position = 0
        
                #Eğim bilgilerini yazdır
                #print(f"Left angle: {self.angle_calculator.find_slope(*left_avg):.2f}, Right angle: {self.angle_calculator.find_slope(*right_avg):.2f}")
                #print(f"Midpoint: ({midpoint_x}, {midpoint_y})")
                self.kayma = midpoint_x-x
        
            cv2.putText(result, f"Angle: {int(steering_angle)} degrees", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(result, f"Lane: {self.lane_position}", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            return result,edges

    
    def bird_eye_view(self,frame):
        #Todo: Kuşbakışı görünüm dönüşümü uygular ve şerit çizgilerinin açılarını hesaplar
        bird=self.bird_eye.transform(frame)
        #Todo: Şerit çizgilerinin kenarlarını ortaya çıkarır
        bird_canny=self.roi_object.edge_detection(bird)
        #Todo: Hough Transform ile doğru parçalarını bulma
        bird_lines=cv2.HoughLinesP(bird_canny, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=50)

        angle = None


        #Todo: Bulunan çizgilerin açılarını hesapla ve çiz
        if bird_lines is not None:
                for line in bird_lines:
                    x1, y1, x2, y2 = line[0]
                    #Todo: Eğim bilgisine göre açı hesapla
                    slope = self.angle_calculator.find_slope(x1, y1, x2, y2)
                    #Todo: Eğimden açı hesapla
                    angle = self.angle_calculator.slope_to_angle(slope)
                    #print("Bird eye:"+str(angle))
                    #Todo: Çizgiyi çiz
                    cv2.line(bird, (x1, y1), (x2, y2), (0, 0, 255), thickness=5)
                    #print("suan kusbakis icindeyiz angle"+str(angle))

        return bird, bird_canny, angle
    
    

    def station_bird_eye_view(self, frame):
        #Todo: Özel duraklar için kuşbakışı görünüm dönüşümü uygular ve şerit çizgilerinin açılarını hesaplar
        station_bird = self.station_bird_eye.transform(frame)
        #Todo: Şerit çizgilerinin kenarlarını ortaya çıkarır
        station_edges = self.roi_object.edge_detection(station_bird)
        #Todo: Hough Transform ile doğru parçalarını bulma
        station_lines = cv2.HoughLinesP(station_edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=50)

        angle = None
        #Todo: Bulunan çizgilerin açılarını hesapla ve çiz
        if station_lines is not None:
            #Todo: Bulunan çizgilerin açılarını hesapla ve çiz
            for line in station_lines:
                #Todo: Eğim bilgisine göre açı hesapla
                x1, y1, x2, y2 = line[0]
                #Todo: Eğim bilgisine göre açı hesapla
                slope = self.angle_calculator.find_slope(x1, y1, x2, y2)
                #Todo: Eğimden açı hesapla
                angle = self.angle_calculator.slope_to_angle(slope)
                #todo:Çizgiyi çiz
                cv2.line(station_bird, (x1, y1), (x2, y2), (0, 0, 255), thickness=5)

        return station_bird, angle 