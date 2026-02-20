import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int32, Bool
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

if ROS2_AVAILABLE:
    class JoystickNode(Node):
        def __init__(self):
            super().__init__('joystick_node')
            
            self.declare_parameter('port', '/dev/ttyUSB1')
            self.declare_parameter('baudrate', 9600)
            
            port = self.get_parameter('port').value
            baudrate = self.get_parameter('baudrate').value
            
            try:
                import serial
                self.arduino = serial.Serial(port, baudrate, timeout=1)
                time.sleep(2)
                self.get_logger().info(f'Kumanda Arduino bağlandı: {port} @ {baudrate}')
            except Exception as e:
                self.get_logger().error(f'Kumanda Arduino bağlantı hatası: {e}')
                raise
            
            self.ileri_geri_pub = self.create_publisher(Int32, '/joystick/ileri_geri', 10)
            self.sag_sol_pub = self.create_publisher(Int32, '/joystick/sag_sol', 10)
            self.vites_pub = self.create_publisher(Int32, '/joystick/vites', 10)
            self.mode_pub = self.create_publisher(Bool, '/joystick/manual_mode', 10)
            
            self.timer = self.create_timer(0.02, self.read_joystick)
            
            self.get_logger().info('Joystick Node başlatıldı - 50Hz')
        
        def read_joystick(self):
            """Kumanda Arduino'sundan veri oku ve ROS2'ye yayınla"""
            try:
                self.arduino.write(b"J\n")
                
                self.arduino.reset_input_buffer()
                data = self.arduino.readline().decode(errors="ignore").strip()
                
                if data and data.startswith('m '):
                    parts = data.split()
                    
                    if len(parts) == 5:
                        try:
                            ileri_geri = int(parts[1])
                            sag_sol = int(parts[2])
                            vites = int(parts[3])
                            mode = int(parts[4])
                            
                            ileri_geri_msg = Int32()
                            ileri_geri_msg.data = ileri_geri
                            self.ileri_geri_pub.publish(ileri_geri_msg)
                            
                            sag_sol_msg = Int32()
                            sag_sol_msg.data = sag_sol
                            self.sag_sol_pub.publish(sag_sol_msg)
                            
                            vites_msg = Int32()
                            vites_msg.data = vites
                            self.vites_pub.publish(vites_msg)
                            
                            mode_msg = Bool()
                            mode_msg.data = (mode == 1)
                            self.mode_pub.publish(mode_msg)
                            
                        except ValueError as e:
                            self.get_logger().warning(f'Veri parse hatası: {e}')
                    else:
                        self.get_logger().warning(f'Yanlış format: {data}')
                        
            except Exception as e:
                self.get_logger().warning(f'Joystick okuma hatası: {e}')


    def main(args=None):
        rclpy.init(args=args)
        node = JoystickNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Klavyeden durduruldu')
        finally:
            node.destroy_node()
            rclpy.shutdown()


#Todo: ROS2 yoksa bu sınıfı kullanarak testi kullanacağız
class MockJoystickNode:
    def __init__(self, port='/dev/ttyUSB1', baudrate=9600):
        print(f'Kumanda Arduino\'ya bağlanıyor: {port} @ {baudrate}')
        
        try:
            import serial
            self.arduino = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)
            print('Kumanda Arduino bağlandı!')
        except Exception as e:
            print(f'Bağlantı hatası: {e}')
            print('  Simülasyon modunda devam...')
            self.arduino = None
        
        print('\n Kumanda verisi okunuyor (Ctrl+C ile durdurun)\n')
        print('='*70)
    
    def read_joystick(self):
        try:
            if self.arduino is None:
                ileri_geri = 50
                sag_sol = 0
                vites = 2
                mode = 1
            else:
                self.arduino.write(b"J\n")
                self.arduino.reset_input_buffer()
                data = self.arduino.readline().decode(errors="ignore").strip()
                
                if data and data.startswith('m '):
                    parts = data.split()
                    if len(parts) == 5:
                        ileri_geri = int(parts[1])
                        sag_sol = int(parts[2])
                        vites = int(parts[3])
                        mode = int(parts[4])
                    else:
                        return
                else:
                    return
            
            mode_str = "MANUEL" if mode == 1 else "OTONOM"
            print(f'İleri-Geri: {ileri_geri:4d} | Sağ-Sol: {sag_sol:4d} | '
                  f'Vites: {vites} | Mod: {mode_str}', end='')
            print(f' | Topics: /joystick/ileri_geri={ileri_geri} '
                  f'/joystick/sag_sol={sag_sol} /joystick/vites={vites} '
                  f'/joystick/manual_mode={mode == 1}')
                  
        except Exception as e:
            print(f' Okuma hatası: {e}')
    
    def run(self, frequency=50):
        period = 1.0 / frequency
        print(f'Çalışma frekansı: {frequency} Hz ({period*1000:.1f} ms)\n')
        
        iteration = 0
        try:
            while True:
                iteration += 1
                if iteration % 50 == 0:
                    print(f'\n{"="*70}\n📊 {iteration} okuma tamamlandı\n{"="*70}\n')
                
                self.read_joystick()
                time.sleep(period)
                
        except KeyboardInterrupt:
            print(f'\n\n{"="*70}')
            print('Test durduruldu')
            print(f' Toplam {iteration} okuma yapıldı')
            print('='*70)


def test_main():
    print('\n' + ' JOYSTICK NODE TEST - ROS2 Simülasyonu '.center(70, '=') + '\n')
    
    port = input('Kumanda Arduino portu [/dev/ttyUSB1]: ').strip() or '/dev/ttyUSB1'
    
    node = MockJoystickNode(port=port, baudrate=9600)
    node.run(frequency=50)


if __name__ == '__main__':
    try:
        import rclpy
        main()
    except ImportError:
        print('⚠️  ROS2 (rclpy) bulunamadı - Test modunda çalışıyor\n')
        test_main()