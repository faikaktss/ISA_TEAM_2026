#!/usr/bin/env python3
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int32, Bool
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    
import time

if ROS2_AVAILABLE:
    class TeensyNode(Node):
        def __init__(self):
            super().__init__('teensy_node')
            
            #Todo:  Parametreler - serial port ayarları
            self.declare_parameter('teensy_port', '/dev/ttyACM0')
            self.declare_parameter('teensy_baudrate', 9600)
            teensy_port = self.get_parameter('teensy_port').value
            teensy_baudrate = self.get_parameter('teensy_baudrate').value
            
            #Todo: Teensy bağlantısı
            try:
                self.teensy = serial.Serial(teensy_port, teensy_baudrate, timeout=1)
                time.sleep(2)  
                self.get_logger().info(f'Teensy bağlandı: {teensy_port} @ {teensy_baudrate}')
            except Exception as e:
                self.teensy = None
                self.get_logger().warning(f'Teensy bağlanamadı: {e} - TEST MODUNDA DEVAM')
            
            # Todo: Durum değişkenleri - MANUEL MOD
            self.joystick_ileri_geri = 0
            self.joystick_sag_sol = 0
            self.joystick_vites = 0
            
            # Todo: Durum değişkenleri - OTONOM MOD
            self.control_ileri_geri = 50
            self.control_sag_sol = 0
            self.control_vites = 0
            
            # Todo: Mod kontrolü 
            self.manual_mode = False
            
            # Todo: Subscriber'lar - MANUEL MOD 
            self.joystick_ileri_geri_sub = self.create_subscription(
                Int32, '/joystick/ileri_geri', self.joystick_ileri_geri_callback, 10)
            self.joystick_sag_sol_sub = self.create_subscription(
                Int32, '/joystick/sag_sol', self.joystick_sag_sol_callback, 10)
            self.joystick_vites_sub = self.create_subscription(
                Int32, '/joystick/vites', self.joystick_vites_callback, 10)
            self.manual_mode_sub = self.create_subscription(
                Bool, '/joystick/manual_mode', self.manual_mode_callback, 10)
            
            # Todo: Subscriber'lar - OTONOM MOD 
            self.control_ileri_geri_sub = self.create_subscription(
                Int32, '/control/ileri_geri', self.control_ileri_geri_callback, 10)
            self.control_sag_sol_sub = self.create_subscription(
                Int32, '/control/sag_sol', self.control_sag_sol_callback, 10)
            self.control_vites_sub = self.create_subscription(
                Int32, '/control/vites', self.control_vites_callback, 10)
            
            # Todo: Timer - 50 Hz (0.02 saniye)
            self.timer = self.create_timer(0.02, self.send_to_teensy)
            
            self.get_logger().info('Teensy node başlatıldı - 50Hz çalışıyor')
        
        # Todo: MANUEL MOD Callback'leri
        def joystick_ileri_geri_callback(self, msg):
            self.joystick_ileri_geri = msg.data
            self.get_logger().debug(f'Joystick ileri_geri: {msg.data}')
        
        def joystick_sag_sol_callback(self, msg):
            self.joystick_sag_sol = msg.data
            self.get_logger().debug(f'Joystick sag_sol: {msg.data}')
        
        def joystick_vites_callback(self, msg):
            self.joystick_vites = msg.data
            self.get_logger().debug(f'Joystick vites: {msg.data}')
        
        def manual_mode_callback(self, msg):
            old_mode = self.manual_mode
            self.manual_mode = msg.data
            if old_mode != self.manual_mode:
                mode_str = "MANUEL" if self.manual_mode else "OTONOM"
                self.get_logger().info(f' Mod değişti: {mode_str}')
        
        # Todo: OTONOM MOD Callback'leri
        def control_ileri_geri_callback(self, msg):
            self.control_ileri_geri = msg.data
            self.get_logger().debug(f'Control ileri_geri: {msg.data}')
        
        def control_sag_sol_callback(self, msg):
            self.control_sag_sol = msg.data
            self.get_logger().debug(f'Control sag_sol: {msg.data}')
        
        def control_vites_callback(self, msg):
            self.control_vites = msg.data
            self.get_logger().debug(f'Control vites: {msg.data}')
        
        # Todo: Ana fonksiyon - Her 50Hz'de çalışır
        def send_to_teensy(self):
            """Moda göre verileri seç ve Teensy'ye gönder"""
            
            if self.manual_mode:
                # Todo: MANUEL MOD  joystick verilerini kullan
                ileri_geri = self.joystick_ileri_geri
                sag_sol = self.joystick_sag_sol
                vites = self.joystick_vites
                mode_indicator = 1  # Manuel
            else:
                # Todo: OTONOM MOD: control verilerini kullan
                ileri_geri = self.control_ileri_geri
                sag_sol = self.control_sag_sol
                vites = self.control_vites
                mode_indicator = 0  #Todo:  Otonom

            otonom = 0 if self.manual_mode else 1
            command = f"{sag_sol},{ileri_geri},{vites},{otonom}\n"
            
            if self.teensy:
                try:
                    self.teensy.write(command.encode())
                    self.get_logger().debug(f'Teensy  {command.strip()}')
                except Exception as e:
                    self.get_logger().error(f'Teensy yazma hatası: {e}')
            else:
                # Test modu
                mode_str = "MANUEL" if self.manual_mode else "OTONOM"
                self.get_logger().debug(
                    f'[TEST-{mode_str}] Teensy komut: {command.strip()} | '
                    f'ileri={ileri_geri}, sol={sag_sol}, vites={vites}'
                )
        
        def destroy_node(self):
            """Node kapanırken Teensy bağlantısını kapat"""
            if self.teensy:
                self.teensy.close()
                self.get_logger().info('Teensy bağlantısı kapatıldı')
            super().destroy_node()


# Todo: Test modu için Mock sınıfı
class MockTeensyNode:
    """ROS2 olmadan test için basit sınıf"""
    def __init__(self):
        print(" ROS2 (rclpy) bulunamadı - Test modunda çalışıyor")
        print("=" * 60)

        # Todo: Sabit test değerleri
        self.manual_mode = False
        self.ileri_geri = 50
        self.sag_sol = 0
        self.vites = 0
        self.counter = 0

        try:
            self.teensy = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            time.sleep(2)
            print("Teensy /dev/ttyACM0 portuna BAGLANDI!")
        except Exception as e:
            self.teensy = None
            print(f"Teensy baglanilamadi: {e}")
    
    def run(self):
        """Test verilerini göster"""
        try:
            while True:
                self.counter += 1
                
                # Todo:Her 5 saniyede mod değiştir
                if self.counter % 250 == 0:
                    self.manual_mode = not self.manual_mode
                
                # Todo,: 2 saniyede açı değiştir
                if self.counter % 100 == 0:
                    self.sag_sol = (self.sag_sol + 10) % 40 - 20
                
                mode_str = "MANUEL" if self.manual_mode else "OTONOM"
                command = f"{self.sag_sol}\n"

                if self.teensy:
                    self.teensy.write(command.encode())

                print(f"[{mode_str}] Teensy komut: {command.strip()} | "
                      f"ileri={self.ileri_geri}, sol={self.sag_sol}, vites={self.vites}")
                
                if self.counter % 50 == 0:
                    print(f" {self.counter} komut gönderildi")
                
                time.sleep(0.02)  # Todo : 50Hz
                
        except KeyboardInterrupt:
            print(f"\n Test durduruldu /  Toplam {self.counter} komut gönderildi")


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ROS2 başlatılamadı: rclpy modülü bulunamadı")
        print("Test moduna geçiliyor\n")
        mock = MockTeensyNode()
        mock.run()
        return
    
    try:
        rclpy.init(args=args)
        node = TeensyNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f"ROS2 başlatılamadı: {e}")
        print("Test moduna geçiliyor\n")
        mock = MockTeensyNode()
        mock.run()


if __name__ == '__main__':
    main()