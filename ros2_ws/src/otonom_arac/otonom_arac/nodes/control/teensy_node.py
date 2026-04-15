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
                self.get_logger().info(f'Teensy bağlandı: {teensy_port} @ {teensy_baudrate}')
            except Exception as e:
                self.get_logger().error(f'Teensy bağlanamadı: {e}')
                raise
            
            # Todo: Durum değişkenleri - MANUEL MOD
            self.joystick_ileri_geri = 0
            self.joystick_sag_sol = 0
            self.joystick_vites = 0
            
            # Todo: Durum değişkenleri - OTONOM MOD
            self.control_ileri_geri = 50
            self.control_sag_sol = 0
            self.control_vites = 0
            
            # Güvenli başlangıç: manuel mod (joystick aktif)
            self.manual_mode = True
            self.pc_aci = 0

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
            else:
                # Todo: OTONOM MOD: control verilerini kullan
                ileri_geri = self.control_ileri_geri
                sag_sol = self.control_sag_sol
                vites = self.control_vites

            otonom = 0 if self.manual_mode else 1
            command = f"{sag_sol},{ileri_geri},{vites},{otonom},{self.pc_aci}\n"

            if self.teensy:
                try:
                    self.teensy.write(command.encode())
                    self.get_logger().info(f'Teensy → {command.strip()}')
                except Exception as e:
                    self.get_logger().error(f'Teensy yazma hatası: {e}')
        
        def destroy_node(self):
            """Node kapanırken Teensy bağlantısını kapat"""
            if self.teensy:
                self.teensy.close()
                self.get_logger().info('Teensy bağlantısı kapatıldı')
            super().destroy_node()


def main(args=None):
    if not ROS2_AVAILABLE:
        print('[ERROR] rclpy bulunamadı, teensy_node başlatılamıyor.')
        return
    rclpy.init(args=args)
    node = TeensyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()