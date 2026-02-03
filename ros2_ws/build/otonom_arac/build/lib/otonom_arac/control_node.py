#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32
from sensor_msgs.msg import LaserScan
import time
import serial

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Todo: Subscriber'lar - sensör verilerini al
        self.lane_angle_sub = self.create_subscription(
            Float32, '/lane/angle', self.lane_angle_callback, 10)
        self.lane_offset_sub = self.create_subscription(
            Float32, '/lane/offset', self.lane_offset_callback, 10)
        self.detection_sub = self.create_subscription(
            String, '/detection/objects', self.detection_callback, 10)
        self.detection_distance_sub = self.create_subscription(
            Float32, '/detection/distance', self.detection_distance_callback, 10)
        self.lidar_scan_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.lidar_callback, 10)
        
        # Todo: Publisher'lar - motor komutları
        self.servo_pub = self.create_publisher(Int32, '/control/servo', 10)
        self.speed_pub = self.create_publisher(Int32, '/control/speed', 10)
        
        # Todo: Durum değişkenleri
        self.current_angle = None
        self.current_offset = None
        self.current_tabela = None
        self.tabela_distance = 0.0
        self.engel_durumu = 0  # 0=yok, 1=sabit, 2=hareketli
        
        # Todo: Tabela geçmişi (5 kez aynı tabelayı görmek için)
        self.tabela_gecmisi = []
        self.yeni_tab = None
        self.durakPixel = 0
        
        # Todo: Sol/sağ kırmızı çizgi sayacı (girilmez/sağ tabelaları için)
        self.solRedCount = 0
        self.sagRedCount = 0
        
        # Todo: Test mode için Teensy ve Arduino bağlantısı
        self.test_mode = True  # Gerçek donanım yoksa True
        try:
            if not self.test_mode:
                self.teensy = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
                self.arduino = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
                self.get_logger().info('Teensy ve Arduino bağlandı')
            else:
                self.teensy = None
                self.arduino = None
                self.get_logger().info('Test modu - Teensy/Arduino simüle ediliyor')
        except Exception as e:
            self.get_logger().warning(f'Seri port açılamadı: {e}, test modunda devam')
            self.test_mode = True
            self.teensy = None
            self.arduino = None
        
        # Todo: Ana kontrol döngüsü - 10 Hz
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Control node başlatıldı')
    
    def lane_angle_callback(self, msg):
        self.current_angle = int(msg.data)
    
    def lane_offset_callback(self, msg):
        self.current_offset = msg.data
    
    def detection_callback(self, msg):
        self.current_tabela = msg.data if msg.data != "None" else None
        
        # Todo: Tabela geçmişine ekle
        if self.current_tabela is not None:
            self.tabela_gecmisi.append(self.current_tabela)
            if len(self.tabela_gecmisi) > 5:
                self.tabela_gecmisi.pop(0)
        
        #Todo: Son 5 tabela aynıysa yeni komut olarak işaretle
        if (len(self.tabela_gecmisi) == 5 and 
            all(x is not None and x == self.tabela_gecmisi[0] for x in self.tabela_gecmisi)):
            if self.yeni_tab != self.tabela_gecmisi[0]:
                self.yeni_tab = self.tabela_gecmisi[0]
                self.get_logger().info(f'{self.yeni_tab} komutu başlatılıyor (5 kez algılandı)')
    
    def detection_distance_callback(self, msg):
        self.tabela_distance = msg.data
        # Todo: Durak tabelası için pixel sayısı simülasyonu
        if self.current_tabela == "durak":
            self.durakPixel = int(1000.0 / max(self.tabela_distance, 0.5))
    
    def lidar_callback(self, msg):
        # Todo: Lidar verilerinden engel tespiti
        # Todo: Önde 30 derece açıda < 0.5m mesafe varsa engel var
        ranges = msg.ranges
        if len(ranges) == 0:
            return
        
        # Todo: Ön bölge kontrolü (-15 ile +15 derece arası)
        front_indices = list(range(len(ranges) // 2 - 15, len(ranges) // 2 + 15))
        front_ranges = [ranges[i] for i in front_indices if i < len(ranges)]
        
        min_distance = min([r for r in front_ranges if r > 0.0], default=10.0)
        
        if min_distance < 0.3:  # 30 cm'den yakın
            self.engel_durumu = 1  # Sabit engel varsay
        elif min_distance < 0.5:  # 50 cm'den yakın
            # Todo: Hareketli engel tespiti için daha karmaşık mantık gerekebilir
            self.engel_durumu = 2  # Hareketli engel olabilir
        else:
            self.engel_durumu = 0  # Engel yok
    
    def send_teensy_command(self, value):
        """Teensy'ye servo komutu gönder"""
        if self.test_mode:
            # Todo:Test modunda sadece log yaz
            servo_msg = Int32()
            servo_msg.data = int(value)
            self.servo_pub.publish(servo_msg)
            self.get_logger().debug(f'[TEST] Teensy komut: {value}')
        else:
            try:
                self.teensy.write(str(value).encode())
                servo_msg = Int32()
                servo_msg.data = int(value)
                self.servo_pub.publish(servo_msg)
            except Exception as e:
                self.get_logger().error(f'Teensy yazma hatası: {e}')
    
    def control_loop(self):
        
        # Todo: 1. Engel kontrolü - EN YÜKSEK ÖNCELİK
        if self.engel_durumu == 2:  # Hareketli engel
            self.get_logger().info('Hareketli engel - DUR!')
            self.send_teensy_command(100)  # Dur komutu
            return
        
        elif self.engel_durumu == 1:  # Sabit engel - kaçınma manevrası
            self.get_logger().info('Sabit engel - kaçınma manevrası')
            # Sola dön
            for _ in range(30):  # 3 saniye
                self.send_teensy_command(-30)
                time.sleep(0.1)
            # Sağa dön
            for _ in range(30):  # 3 saniye
                self.send_teensy_command(30)
                time.sleep(0.1)
            # İleri git
            for _ in range(67):  # 6.7 saniye
                self.send_teensy_command(25)
                time.sleep(0.1)
            self.engel_durumu = 0  # Manevradan sonra engel yok say
            return
        
        # Todo: 2. Tabela komutları
        if self.yeni_tab == "durak":
            self.get_logger().info(f'Durak yaklaşıyor - pixel: {self.durakPixel}')
            # Durağa yaklaş
            while self.durakPixel < 1200:
                if self.current_angle is not None:
                    self.send_teensy_command(self.current_angle)
                time.sleep(0.1)
                self.get_logger().info(f'Durak bekleniyor: {self.durakPixel}')
            
            # Durak manevraları
            for _ in range(44):  # 4.4 saniye sağa dön
                self.send_teensy_command(20)
                time.sleep(0.1)
            for _ in range(40):  # 4 saniye şerit takibi
                if self.current_angle is not None:
                    self.send_teensy_command(self.current_angle)
                time.sleep(0.1)
            for _ in range(35):  # 3.5 saniye dur
                self.send_teensy_command(100)
                time.sleep(0.2)
            for _ in range(46):  # 4.6 saniye sola dön
                self.send_teensy_command(-23)
                time.sleep(0.1)
            
            self.yeni_tab = None
            self.tabela_gecmisi.clear()
            return
        
        elif self.yeni_tab == "dur":
            self.get_logger().info('DUR tabelası - 8 saniye duruyorum')
            for _ in range(40):  # 8 saniye dur
                self.send_teensy_command(100)
                time.sleep(0.2)
            self.yeni_tab = None
            self.tabela_gecmisi.clear()
            return
        
        elif self.yeni_tab == "girilmez":
            if self.solRedCount >= 10:
                self.get_logger().info('GİRİLMEZ - Sola dönüyorum')
                for _ in range(50):  # 5 saniye sola dön
                    self.send_teensy_command(-20)
                    time.sleep(0.1)
                for _ in range(40):  # 4 saniye şerit takibi
                    if self.current_angle is not None:
                        self.send_teensy_command(self.current_angle)
                    time.sleep(0.1)
                self.yeni_tab = None
                self.tabela_gecmisi.clear()
            return
        
        elif self.yeni_tab == "sag":
            if self.sagRedCount >= 0:
                self.get_logger().info('SAĞ tabelası - Sağa dönüyorum')
                for _ in range(50):  # 5 saniye sağa dön
                    self.send_teensy_command(20)
                    time.sleep(0.1)
                for _ in range(40):  # 4 saniye şerit takibi
                    if self.current_angle is not None:
                        self.send_teensy_command(self.current_angle)
                    time.sleep(0.1)
                self.yeni_tab = None
                self.tabela_gecmisi.clear()
            return
        
        elif self.current_tabela == "park_yasakiki":
            self.get_logger().info('PARK YASAK - Bekliyorum')
            # Todo: Sol yasak görünene kadar dur
            for _ in range(100):  # Max 10 saniye bekle
                if self.current_tabela == "sol_yasak":
                    break
                self.send_teensy_command(100)
                time.sleep(0.1)
            self.yeni_tab = None
            self.tabela_gecmisi.clear()
            return
        
        #Todo: 3. Normal şerit takibi
        if self.current_angle is not None:
            self.send_teensy_command(self.current_angle)
        else:
            self.get_logger().debug('Açı verisi yok, bekliyorum')
    
    def destroy_node(self):
        if self.teensy:
            self.teensy.close()
        if self.arduino:
            self.arduino.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
