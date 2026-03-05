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
    def __init__(self, COM='/dev/ttyUSB0', baudrate=115200, timeout=1):
        self.ser = serial.Serial(COM, baudrate, timeout=timeout)
        self.ilk_mesafe = None

    def setValue(self, value):
        if value is not None:
            value = str(value)
            self.ser.reset_output_buffer()
            self.ser.write(value.encode())
        
    def getValue(self):
        self.ser.reset_input_buffer()
        data = self.ser.readline().decode(errors="ignore").strip()
        return data
    
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
                time.sleep(2)
                self.get_logger().info(f'Arduino Encoder bağlandı: {port} @ {baudrate}')
            except Exception as e:
                self.get_logger().error(f'Arduino Encoder bağlantı hatası: {e}')
                raise

            #Todo: Veri yayınlanacak topicler
            self.distance_pub = self.create_publisher(Int32 , '/encoder/distance',10)
            self.speed_pub = self.create_publisher(Float32,'/encoder/speed',10)


            #Todo: Hız hesabı için değişkenler(faik)
            self.last_distance = 0
            self.last_time = time.time()

            self.timer = self.create_timer(0.02,self.read_encoder)

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

                        speed_msg = Float32()
                        speed_msg.data = float(speed)
                        self.speed_pub.publish(speed_msg)


                    self.last_distance = distance
                    self.last_time = current_time

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
            rclpy.shutdown()



#Todo: ROS2 OLMADAN TEST AŞAMASI ------------(FAİK)
class MockEncoderNode:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        print(f'Arduino ya bağlanıyor: {port} @ {baudrate}')

        try:
            self.arduino = Arduino(COM=port, baudrate=baudrate, timeout=1)
            time.sleep(2)
            print(f'Arduino Encoder bağlandı')

        except Exception as e:
            print(f'Arduino Encoder bağlantı hatası: {e}')
            print('Arduino bağlantısı başarısız, simülasyona geçiliyor')
            self.arduino = None

        self.last_distance = 0
        self.last_time = time.time()

        print('Veri okuma başladı ')
        print('='*60)

    def read_encoder(self):
        try:
            if self.arduino is None:
                distance = self.last_distance + 23
            else:
                distance = self.arduino.encoder_distance()

            if distance is not None:
                print(f'Mesafe:{distance:6d} mm ',end= "")

                current_time = time.time()
                time_diff = current_time - self.last_time

                if time_diff > 0:
                    speed = (distance - self.last_distance) / time_diff
                    print(f'Hız: {speed:6.2f} mm/s',end="")
                    print(f'Topics : /encode/distance={distance} /encoder/speed={speed:6.2f}',end="")

                self.last_distance = distance
                self.last_time = current_time
            else:
                print('Veri alınamadı ')

        except Exception as e:
            print(f'Encoder okuma hatası: {e}')

    def run(self, frequency=50):
        """50Hz ile sürekli veri oku"""
        period = 1.0 / frequency
        print(f" Çalışma frekansı: {frequency} Hz ({period*1000:.1f} ms)\n")
        
        iteration = 0
        try:
            while True:
                iteration += 1
                if iteration % 50 == 0:
                    print(f"\n{'='*60}\n {iteration} okuma tamamlandı\n{'='*60}\n")
                
                self.read_encoder()
                time.sleep(period)
                
        except KeyboardInterrupt:
            print(f"{'='*60}")
            print(" Test durduruldu")
            print(f" Toplam {iteration} okuma yapıldı")
            print("="*60)


def test_main():
    """Test modu için ana fonksiyon"""
    print(" ENCODER NODE TEST - ROS2 Simülasyonu ".center(60, "=") + "\n")
    
    port = input("Arduino portu [/dev/ttyUSB0]: ").strip() or '/dev/ttyUSB0'
    
    node = MockEncoderNode(port=port, baudrate=115200)
    node.run(frequency=50)


if __name__ == '__main__':
    try:
        import rclpy
        main()
    except ImportError:
        print("⚠️  ROS2 (rclpy) bulunamadı - Test modunda çalışıyor\n")
        test_main()