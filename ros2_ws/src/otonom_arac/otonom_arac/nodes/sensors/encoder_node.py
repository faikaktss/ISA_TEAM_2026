import time
import serial

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int32, Float32
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    pass


# ============================================================================
# Arduino Sınıfı - Encoder iletişimi için
# ============================================================================
class Arduino:
    """Arduino ile seri port üzerinden iletişim kurar ve encoder verisini okur"""
    def __init__(self, COM='/dev/ttyUSB', baudrate=115200, timeout=1):
        self.ser = serial.Serial(COM, baudrate, timeout=0.015)  # 50Hz timer bütçesi 20ms; 1s timeout timer'ı blokluyordu
        self.ilk_mesafe = None

    def setValue(self, value):
        if value is not None:
            value = str(value)
            self.ser.reset_output_buffer()
            self.ser.write(value.encode())
        
    def getValue(self):
        # Son tam satırı oku (reset_input_buffer veri kaybına yol açıyordu)
        last_valid = None
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line:
                last_valid = line
        if last_valid is None:
            last_valid = self.ser.readline().decode(errors="ignore").strip()
        return last_valid
    
    def encoder_distance(self):
        """Encoder'dan mesafe verisi alır"""
        self.setValue("1000")
        distance = self.getValue()
        try:
            return int(distance)
        except ValueError:
            return None

if ROS2_AVAILABLE:
    #Todo: Bu kısım arduino'dan veri okur 
    class EncoderNode(Node):
        def __init__(self):
            #Todo: Ros2 node'u başlat
            super().__init__('encoder_node')

            #Todo: Ros2 için parametreler(faik)
            self.declare_parameter('port','/dev/ttyUSB0')
            self.declare_parameter('baudrate',115200)

            port = self.get_parameter('port').value
            baudrate = self.get_parameter('baudrate').value

            #Todo: Ardunio bağlantısı
            try:
                self.arduino =Arduino(COM=port, baudrate=baudrate,timeout=1)
                self.get_logger().info(f'Arduino Encoder bağlandı: {port} @ {baudrate}')
                # TERMINAL: başlangıç logu
                print(f'[ENCODER] Başlatıldı | Port: {port} @ {baudrate}', flush=True)
            except Exception as e:
                self.get_logger().error(f'Arduino Encoder bağlantı hatası: {e}')
                # TERMINAL: bağlantı hatası
                print(f'[ENCODER] ✗ Port açılamadı: {port} — {e}', flush=True)
                raise

            #Todo: Veri yayınlanacak topicler
            self.distance_pub = self.create_publisher(Int32 , '/encoder/distance',10)
            self.speed_pub = self.create_publisher(Float32,'/encoder/speed',10)


            #Todo: Hız hesabı için değişkenler(faik)
            self.last_distance = 0
            self.last_time = time.time()

            self.timer = self.create_timer(0.02,self.read_encoder)
            # TERMINAL: 50 çağrıda bir log sayacı
            self._enc_log_count = 0
            self._last_speed = 0.0
            self._no_data_count = 0

            self.get_logger().info('Encoder Node başlatıldı - 50Hz')

        def read_encoder(self):
            #Todo: Artık ardunio'dan encoder mesafe verisini okuyup ros2 ile yayınlayacağız
            try:
                distance = self.arduino.encoder_distance()
                if distance is not None:
                    distance_msg = Int32()
                    distance_msg.data = distance
                    self.distance_pub.publish(distance_msg)

                    current_time = time.time()
                    time_diff = current_time - self.last_time

                    if time_diff > 0:
                        speed=(distance - self.last_distance) /time_diff
                        # TERMINAL: hız güncelle
                        self._last_speed = float(speed)
                        self._no_data_count = 0

                        speed_msg = Float32()
                        speed_msg.data = float(speed)
                        self.speed_pub.publish(speed_msg)


                    self.last_distance = distance
                    self.last_time = current_time

                    # TERMINAL: her 50 çağrıda bir (~1s) log bas
                    self._enc_log_count += 1
                    if self._enc_log_count >= 50:
                        print(
                            f'[ENCODER] mesafe={distance} mm | hız={self._last_speed:.1f} mm/s',
                            flush=True)
                        self._enc_log_count = 0
                else:
                    # TERMINAL: veri gelmiyorsa sayı
                    self._no_data_count += 1

            except Exception as e:
                self.get_logger().warning(f'Encoder okuma hatası: {e}')

    def main(args=None):
        #Todo: Ana fonksiyon node'u başlat
        rclpy.init(args=args)

        node = EncoderNode()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Klavyeden durduruldu ')
        finally:
            node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass



if __name__ == '__main__':
    main()