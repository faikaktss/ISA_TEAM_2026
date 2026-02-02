# Todo: Robotun tüm sensör ve verilerini saklamak için kullanılan bilgi sınıfı
# Todo: Karar mekanizması bu sınıf üzerinden verilere erişir ve günceller
class Info:
    line = None
    def __init__(self):
        self._angle = None          # Todo: Robot'un açısı (derece cinsinden)
        self._line = None           # Todo: LIDAR tarafından algılanan çizgi
        self._tabela = None         # Todo: İşaret tablosu verisi
        self._encoder = None        # Todo: Tekerlek dönüş sayıcısı (pozisyon)
        self._imu = None            # Todo: Hızlandırma ve dönüş sensörü
        self._gps = None            # Todo: Konum bilgisi
        self._name = None           # Todo: Robot/görev adı
        self._description = None    # Todo: Açıklama metni
        self._lane = None           # Todo: Şerit bilgisi (şerit algılama)
        self._originalFrame = None  # Todo: Kameranın orijinal görüntüsü
        self._birdEyeFrame = None   # Todo: Kuşbaşı görünüme dönüştürülmüş görüntü
        self._distance = None       # Todo: Mesafe ölçümü
        self._engel = None          # Todo: Engel sağda mı solda mı (YML yorum)
        self._edgeBirdEye = None    # Todo: Kuşbaşı görünümdeki kenarlar
        self.durakPixel = 0         # Todo: Durağın pixel konumu
        self._solRedCount = None    # Todo: Sol tarafta kırmızı renkli piksellerin sayısı
        self._sagRedCount = None    # Todo: Sağ tarafta kırmızı renkli piksellerin sayısı
        self._durak_engel = None    # Todo: Durakta engel var mı?

    def set_angle(self, angle):
        self._angle = angle

    def get_angle(self):
        return self._angle

    #lidar
    def set_line(self, line):
        self._line = line
        
    def get_line(self):
      return self._line

    def set_tabela(self, tabela):
        self._tabela = tabela
    
    def get_tabela(self):
        return self._tabela
    
    def set_encoder(self,encoder):
        self._encoder = encoder

    def get_encoder(self):
        return self._encoder
    
    def set_imu(self,imu):
        self._imu = imu

    def get_imu(self):
        return self._imu
    
    def set_gps(self,gps):
        self._gps = gps
    
    def get_gps(self):
        return self._gps
    
    def set_name(self,name):
        self._name = name
    
    def get_name(self):
        return self._name
    
    def set_description(self,description):
        self._description = description
    
    def get_description(self):
        return self._description
    
    def set_lane(self,lane):
        self._lane = lane
    
    def get_lane(self):
        return self._lane
    
    def set_originalFrame(self,originalFrame):
        self._originalFrame = originalFrame
    
    def get_originalFrame(self):
        return self._originalFrame
    
    def set_birdEyeFrame(self,birdEyeFrame):
        self._birdEyeFrame = birdEyeFrame
    
    def get_birdEyeFrame(self):
        return self._birdEyeFrame
    
    def set_distance(self,distance):
        self._distance = distance
    
    def get_distance(self):
        return self._distance
    
    def set_engel(self,engel):
        self._engel = engel
    
    def get_engel(self):
        return self._engel
        
    def set_edgeBirdEye(self,edgeBirdEye):
        self._edgeBirdEye = edgeBirdEye
    
    def get_edgeBirdEye(self):
        return self._edgeBirdEye
    
    def set_durakPixel(self,v):
        self.durakPixel = v

    def get_durakPixel(self):
        return self.durakPixel

    def set_solRedCount(self,solRedCount):
        self._solRedCount = solRedCount
    
    def get_solRedCount(self):
        return self._solRedCount
    
    def set_sagRedCount(self,sagRedCount):
        self._sagRedCount = sagRedCount
    
    def get_sagRedCount(self):
        return self._sagRedCount
    
    def set_durak_engel(self,durak_engel):
        self._durak_engel = durak_engel

    def get_durak_engel(self):
        return self._durak_engel

