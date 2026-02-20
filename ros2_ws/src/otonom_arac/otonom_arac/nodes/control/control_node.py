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
        
        # Todo: Parametreler - seri port ayarları
        self.declare_parameter('teensy_port', '/dev/ttyACM0')
        self.declare_parameter('teensy_baudrate', 9600)
        self.declare_parameter('arduino_port', '/dev/ttyACM1')
        self.declare_parameter('arduino_baudrate', 115200)
        self.declare_parameter('test_mode', True)  # Gerçek donanım yoksa True
        
        teensy_port = self.get_parameter('teensy_port').value
        teensy_baudrate = self.get_parameter('teensy_baudrate').value
        arduino_port = self.get_parameter('arduino_port').value
        arduino_baudrate = self.get_parameter('arduino_baudrate').value
        self.test_mode = self.get_parameter('test_mode').value
        
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
        
        # Todo: Publisher'lar  (motor komutları)
        self.sag_sol_pub = self.create_publisher(Int32, '/control/sag_sol', 10)
        self.ileri_geri_pub = self.create_publisher(Int32, '/control/ileri_geri', 10)
        self.vites_pub = self.create_publisher(Int32, '/control/vites', 10)
        
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
        
        # Todo: Teensy ve Arduino bağlantısı
        try:
            if not self.test_mode:
                self.teensy = serial.Serial(teensy_port, teensy_baudrate, timeout=1)
                self.arduino = serial.Serial(arduino_port, arduino_baudrate, timeout=1)
                self.get_logger().info(f'Teensy ({teensy_port}) ve Arduino ({arduino_port}) bağlandı')
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
    
    def send_control_command(self, sag_sol, ileri_geri=50, vites=0):
        """Kontrol komutlarını topic'lere publish et"""
        # Sağ-sol (direksiyon açısı)
        sag_sol_msg = Int32()
        sag_sol_msg.data = int(sag_sol)
        self.sag_sol_pub.publish(sag_sol_msg)
        
        # İleri-geri (motor hızı)
        ileri_geri_msg = Int32()
        ileri_geri_msg.data = int(ileri_geri)
        self.ileri_geri_pub.publish(ileri_geri_msg)
        
        # Vites (0=ileri, 1=geri)
        vites_msg = Int32()
        vites_msg.data = int(vites)
        self.vites_pub.publish(vites_msg)
        
        if self.test_mode:
            self.get_logger().debug(f'[CONTROL] sag_sol={sag_sol}, ileri_geri={ileri_geri}, vites={vites}')

    def control_loop(self):
        
        # Todo: 1. Engel kontrolü - EN YÜKSEK ÖNCELİK
        if self.engel_durumu == 2:  # Hareketli engel
            self.get_logger().info('Hareketli engel - DUR!')
            self.send_control_command(0, ileri_geri=0, vites=0)  # Dur komutu
            return
        
        elif self.engel_durumu == 1:  # Sabit engel - kaçınma manevrası
            self.get_logger().info('Sabit engel - kaçınma manevrası')
            # Sola dön
            for _ in range(30):  # 3 saniye
                self.send_control_command(-30, ileri_geri=50, vites=0)
                time.sleep(0.1)
            # Sağa dön
            for _ in range(30):  # 3 saniye
                self.send_control_command(30, ileri_geri=50, vites=0)
                time.sleep(0.1)
            # İleri git
            for _ in range(67):  # 6.7 saniye
                self.send_control_command(25, ileri_geri=50, vites=0)
                time.sleep(0.1)
            self.engel_durumu = 0  # Manevradan sonra engel yok say
            return
        
        # Todo: 2. Tabela komutları
        if self.yeni_tab == "durak":
            self.get_logger().info(f'Durak yaklaşıyor - pixel: {self.durakPixel}')
            # Durağa yaklaş
            while self.durakPixel < 1200:
                if self.current_angle is not None:
                    self.send_control_command(self.current_angle, ileri_geri=50, vites=0)
                time.sleep(0.1)
                self.get_logger().info(f'Durak bekleniyor: {self.durakPixel}')
            
            # Durak manevraları
            for _ in range(44):  # 4.4 saniye sağa dön
                self.send_control_command(20, ileri_geri=50, vites=0)
                time.sleep(0.1)
            for _ in range(40):  # 4 saniye şerit takibi
                if self.current_angle is not None:
                    self.send_control_command(self.current_angle, ileri_geri=50, vites=0)
                time.sleep(0.1)
            for _ in range(35):  # 3.5 saniye dur
                self.send_control_command(0, ileri_geri=0, vites=0)
                time.sleep(0.2)
            for _ in range(46):  # 4.6 saniye sola dön
                self.send_control_command(-23, ileri_geri=50, vites=0)
                time.sleep(0.1)
            
            self.yeni_tab = None
            self.tabela_gecmisi.clear()
            return
        
        elif self.yeni_tab == "dur":
            self.get_logger().info('DUR tabelası - 8 saniye duruyorum')
            for _ in range(40):  # 8 saniye dur
                self.send_control_command(0, ileri_geri=0, vites=0)
                time.sleep(0.2)
            self.yeni_tab = None
            self.tabela_gecmisi.clear()
            return
        
        elif self.yeni_tab == "girilmez":
            if self.solRedCount >= 10:
                self.get_logger().info('GİRİLMEZ - Sola dönüyorum')
                for _ in range(50):  # 5 saniye sola dön
                    self.send_control_command(-20, ileri_geri=50, vites=0)
                    time.sleep(0.1)
                for _ in range(40):  # 4 saniye şerit takibi
                    if self.current_angle is not None:
                        self.send_control_command(self.current_angle, ileri_geri=50, vites=0)
                    time.sleep(0.1)
                self.yeni_tab = None
                self.tabela_gecmisi.clear()
            return
        
        elif self.yeni_tab == "sag":
            if self.sagRedCount >= 0:
                self.get_logger().info('SAĞ tabelası - Sağa dönüyorum')
                for _ in range(50):  # 5 saniye sağa dön
                    self.send_control_command(20, ileri_geri=50, vites=0)
                    time.sleep(0.1)
                for _ in range(40):  # 4 saniye şerit takibi
                    if self.current_angle is not None:
                        self.send_control_command(self.current_angle, ileri_geri=50, vites=0)
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
                self.send_control_command(0, ileri_geri=0, vites=0)
                time.sleep(0.1)
            self.yeni_tab = None
            self.tabela_gecmisi.clear()
            return
        
        #Todo: 3. Normal şerit takibi
        if self.current_angle is not None:
            self.send_control_command(self.current_angle, ileri_geri=50, vites=0)
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
