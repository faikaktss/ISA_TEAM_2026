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
                self.arduino.reset_input_buffer()  # startup garbage'ı temizle
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
            """Kumanda Arduino'sundan veri oku ve ROS2'ye yayınla.

            Arduino format: dumen,ileri,vites,otonom
              dumen  = CH1 sag-sol  (0-255, orta=127)
              ileri  = CH3 ileri-geri (0-255)
              vites  = CH7 switch (0 veya 1)
              otonom = CH10 switch (0=manuel, 1=otonom)
            """
            try:
                if self.arduino.in_waiting == 0:
                    return

                data = self.arduino.readline().decode(errors="ignore").strip()

                if not data:
                    return

                parts = data.split(',')

                if len(parts) != 4:
                    self.get_logger().warning(f'Yanlış format (4 değer bekleniyor): {data}')
                    return

                dumen   = int(parts[0])  # CH1 - sag-sol
                ileri   = int(parts[1])  # CH3 - ileri-geri
                vites   = int(parts[2])  # CH7 - vites switch
                otonom  = int(parts[3])  # CH10 - mod switch

                sag_sol_msg = Int32()
                sag_sol_msg.data = dumen
                self.sag_sol_pub.publish(sag_sol_msg)

                ileri_geri_msg = Int32()
                ileri_geri_msg.data = ileri
                self.ileri_geri_pub.publish(ileri_geri_msg)

                vites_msg = Int32()
                vites_msg.data = vites
                self.vites_pub.publish(vites_msg)

                # otonom=0 → manuel mod aktif (True), otonom=1 → otonom mod aktif (False)
                mode_msg = Bool()
                mode_msg.data = (otonom == 0)
                self.mode_pub.publish(mode_msg)

                self.get_logger().info(
                    f'Kumanda → sag_sol={dumen} ileri_geri={ileri} '
                    f'vites={vites} otonom={otonom}'
                )

            except ValueError as e:
                self.get_logger().warning(f'Veri parse hatası: {e} | ham: {data}')
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
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()