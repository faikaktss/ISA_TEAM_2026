import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

#Todo:Lidar kütüphanesini import et (faik)
try:
    from .lidar import LidarSystem
    LIDAR_AVAILABLE = True
except ImportError as e:
    LIDAR_AVAILABLE = False
    print(f" Lidar import hatası: {e}")

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        
        #Todo:Lidar verilerini yayınla
        self.scan_publisher = self.create_publisher(LaserScan, '/lidar/scan', 10)
        self.distance_publisher = self.create_publisher(Float32MultiArray, '/lidar/distances', 10)
        
        self.timer_period = 1.0 / 10.0  #Todo:10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.test_mode = False
        
        if LIDAR_AVAILABLE:
            try:
                #Todo:Lidar'ı başlat (port'u ayarla)
                self.lidar = LidarSystem(port='/dev/ttyUSB0', enable_gui=False, print_clusters=False)
                self.lidar.start()
                self.get_logger().info('Lidar başlatıldı')
            except Exception as e:
                self.get_logger().warn(f'Lidar başlatılamadı: {str(e)}. Test moduna geçiliyor.')
                self.test_mode = True
        else:
            self.get_logger().warn('Lidar kütüphanesi bulunamadı. Test modu aktif.')
            self.test_mode = True
        
        if self.test_mode:
            self.angle_counter = 0
            self.get_logger().info('📡 Test Lidar hazır')
    
    def timer_callback(self):
        try:
            if self.test_mode:
                # Todo:Test modu: Sahte Lidar verisi üret
                scan_msg, distance_msg = self._generate_test_data()
            else:
                #Todo : Gerçek Lidar'dan veri al
                scan_msg, distance_msg = self._get_lidar_data()
            
            if scan_msg:
                self.scan_publisher.publish(scan_msg)
            if distance_msg:
                self.distance_publisher.publish(distance_msg)
                
        except Exception as e:
            self.get_logger().error(f'Lidar okuma hatası: {str(e)}')
    
    def _generate_test_data(self):
        self.angle_counter += 0.1
        
        #Todo : LaserScan mesajı
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = -3.14159  # -180 derece
        scan.angle_max = 3.14159   # +180 derece
        scan.angle_increment = 0.01745  # ~1 derece
        scan.time_increment = 0.0
        scan.range_min = 0.15
        scan.range_max = 12.0
        
        #Todo: Sahte mesafe verileri (sinüs dalgası gibi Faik)
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [1.0 + 0.5 * np.sin(i * 0.1 + self.angle_counter) for i in range(num_readings)]
        
        #Todo:Mesafe dizisi
        distance_msg = Float32MultiArray()
        distance_msg.data = [2.5, 3.0, 2.8, 3.2]  # Örnek: 4 yöndeki mesafeler
        
        return scan, distance_msg
    
    def _get_lidar_data(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_link'
        
        #Todo:Mesafe dizisi
        distance_msg = Float32MultiArray()
        distance_msg.data = [0.0, 0.0, 0.0, 0.0]  # Placeholder
        
        return scan, distance_msg

def main(args=None):
    rclpy.init(args=args)
    lidar_node = LidarNode()
    
    try:
        rclpy.spin(lidar_node)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()