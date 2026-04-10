import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        
        # Todo: Parametreler - Lidar ayarları
        self.declare_parameter('lidar_port', '/dev/ttyUSB1')
        self.declare_parameter('lidar_baudrate', 115200)

        lidar_port = self.get_parameter('lidar_port').value
        lidar_baudrate = self.get_parameter('lidar_baudrate').value

        #Todo:Lidar verilerini yayınla
        self.scan_publisher = self.create_publisher(LaserScan, '/lidar/scan', 10)
        self.distance_publisher = self.create_publisher(Float32MultiArray, '/lidar/distances', 10)

        self.timer_period = 1.0 / 10.0  #Todo:10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Gerçek RPLidar kütüphanesi buraya eklenebilir
        # from adafruit_rplidar import RPLidar
        # self.lidar = RPLidar(None, lidar_port, baudrate=lidar_baudrate)
        self.get_logger().info(f'Lidar bağlandı: {lidar_port} @ {lidar_baudrate}')
        self.lidar = None  # Placeholder
    
    def timer_callback(self):
        try:
            #Todo : Gerçek Lidar'dan veri al
            scan_msg, distance_msg = self._get_lidar_data()

            if scan_msg:
                self.scan_publisher.publish(scan_msg)
            if distance_msg:
                self.distance_publisher.publish(distance_msg)

        except Exception as e:
            self.get_logger().error(f'Lidar okuma hatası: {str(e)}')

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