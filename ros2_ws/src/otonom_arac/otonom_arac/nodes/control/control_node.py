#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32
from sensor_msgs.msg import LaserScan
import time

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter('arduino_port', '/dev/ttyACM1')
        self.declare_parameter('arduino_baudrate', 9600)

        arduino_port = self.get_parameter('arduino_port').value
        arduino_baudrate = self.get_parameter('arduino_baudrate').value

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

        self.sag_sol_pub = self.create_publisher(Int32, '/control/sag_sol', 10)
        self.ileri_geri_pub = self.create_publisher(Int32, '/control/ileri_geri', 10)
        self.vites_pub = self.create_publisher(Int32, '/control/vites', 10)

        self.imu_pub = self.create_publisher(Float32, '/imu/angle', 10)
        self.imu_angle = 0.0

        self.current_angle = None
        self.current_offset = None
        self.current_tabela = None
        self.tabela_distance = 0.0
        self.engel_durumu = 0

        self.tabela_gecmisi = []
        self.yeni_tab = None
        self.durakPixel = 0

        self.solRedCount = 0
        self.sagRedCount = 0

        self.state = 'lane_following'
        self.state_counter = 0

        if SERIAL_AVAILABLE:
            self.arduino = serial.Serial(arduino_port, arduino_baudrate, timeout=1)
            self.get_logger().info(f'Arduino bağlandı: {arduino_port} @ {arduino_baudrate}')
        else:
            self.get_logger().error('pyserial bulunamadı, arduino bağlantısı kurulamıyor')
            self.arduino = None

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.imu_timer = self.create_timer(0.02, self.imu_okuma)

        self.get_logger().info('Control node başlatıldı')

    def imu_okuma(self):
        """Arduino'dan IMU açısını oku ve /imu/angle topic'ine yayınla"""
        if self.arduino is None:
            return
        try:
            while self.arduino.in_waiting > 0:
                line = self.arduino.readline().decode(errors='ignore').strip()
                if line:
                    try:
                        self.imu_angle = float(line)
                        imu_msg = Float32()
                        imu_msg.data = self.imu_angle
                        self.imu_pub.publish(imu_msg)
                    except ValueError:
                        pass
        except Exception as e:
            self.get_logger().warning(f'IMU okuma hatası: {e}')

    def lane_angle_callback(self, msg):
        self.current_angle = int(msg.data)

    def lane_offset_callback(self, msg):
        self.current_offset = msg.data

    def detection_callback(self, msg):
        self.current_tabela = msg.data if msg.data != "None" else None
        if self.current_tabela is not None:
            self.tabela_gecmisi.append(self.current_tabela)
            if len(self.tabela_gecmisi) > 5:
                self.tabela_gecmisi.pop(0)
        if (len(self.tabela_gecmisi) == 5 and
            all(x is not None and x == self.tabela_gecmisi[0] for x in self.tabela_gecmisi)):
            if self.yeni_tab != self.tabela_gecmisi[0]:
                self.yeni_tab = self.tabela_gecmisi[0]
                self.get_logger().info(f'{self.yeni_tab} komutu başlatılıyor (5 kez algılandı)')

    def detection_distance_callback(self, msg):
        self.tabela_distance = msg.data
        if self.current_tabela == "durak":
            self.durakPixel = int(1000.0 / max(self.tabela_distance, 0.5))

    def lidar_callback(self, msg):
        ranges = msg.ranges
        if len(ranges) == 0:
            return
        front_indices = list(range(len(ranges) // 2 - 15, len(ranges) // 2 + 15))
        front_ranges = [ranges[i] for i in front_indices if i < len(ranges)]
        min_distance = min([r for r in front_ranges if r > 0.0], default=10.0)
        if min_distance < 0.3:
            self.engel_durumu = 1
        elif min_distance < 0.5:
            self.engel_durumu = 2
        else:
            self.engel_durumu = 0

    def send_control_command(self, sag_sol, ileri_geri=50, vites=0):
        sag_sol_msg = Int32()
        sag_sol_msg.data = int(sag_sol)
        self.sag_sol_pub.publish(sag_sol_msg)

        ileri_geri_msg = Int32()
        ileri_geri_msg.data = int(ileri_geri)
        self.ileri_geri_pub.publish(ileri_geri_msg)

        vites_msg = Int32()
        vites_msg.data = int(vites)
        self.vites_pub.publish(vites_msg)

        if self.arduino:
            try:
                if ileri_geri == 0:
                    self.arduino.write(b"100\n")
                else:
                    pc_aci = max(-32, min(32, int(sag_sol)))
                    self.arduino.write(f"{pc_aci}\n".encode())
            except Exception as e:
                self.get_logger().warning(f'Arduino yazma hatası: {e}')

    def gecis(self, yeni_state):
        """State geçişi - sayacı sıfırla"""
        self.get_logger().info(f'State: {self.state} → {yeni_state}')
        self.state = yeni_state
        self.state_counter = 0

    def control_loop(self):
        """Her 0.1 saniyede bir çalışır - sadece 1 adım yapar, ASLA uyumaz"""

        self.state_counter += 1

        if self.state == 'lane_following':
            if self.engel_durumu == 2:
                self.get_logger().info('Hareketli engel - DUR!')
                self.send_control_command(0, ileri_geri=0, vites=0)
                return
            elif self.engel_durumu == 1:
                self.get_logger().info('Sabit engel - kaçınma başlıyor')
                self.gecis('engel_kacin_sol')
                return


        if self.state == 'engel_kacin_sol':
            self.send_control_command(-30, ileri_geri=50, vites=0)
            if self.state_counter >= 30: 
                self.gecis('engel_kacin_sag')

        elif self.state == 'engel_kacin_sag':
            self.send_control_command(30, ileri_geri=50, vites=0)
            if self.state_counter >= 30:   
                self.gecis('engel_kacin_ileri')

        elif self.state == 'engel_kacin_ileri':
            self.send_control_command(25, ileri_geri=50, vites=0)
            if self.state_counter >= 67:  
                self.engel_durumu = 0
                self.gecis('lane_following')

        elif self.state == 'durak_yaklas':
            if self.durakPixel >= 1200:
                self.gecis('durak_sag')
            else:
                if self.current_angle is not None:
                    self.send_control_command(self.current_angle, ileri_geri=50, vites=0)
                self.get_logger().info(f'Durak bekleniyor: {self.durakPixel}')

        elif self.state == 'durak_sag':
            self.send_control_command(20, ileri_geri=50, vites=0)
            if self.state_counter >= 44:  
                self.gecis('durak_serit')

        elif self.state == 'durak_serit':
            if self.current_angle is not None:
                self.send_control_command(self.current_angle, ileri_geri=50, vites=0)
            if self.state_counter >= 40:   
                self.gecis('durak_dur')

        elif self.state == 'durak_dur':
            self.send_control_command(0, ileri_geri=0, vites=0)
            if self.state_counter >= 300:   
                self.gecis('durak_sol')

        elif self.state == 'durak_sol':
            self.send_control_command(-23, ileri_geri=50, vites=0)
            if self.state_counter >= 46:   
                self.yeni_tab = None
                self.tabela_gecmisi.clear()
                self.gecis('lane_following')

        elif self.state == 'dur_bekle':
            self.send_control_command(0, ileri_geri=0, vites=0)
            if self.state_counter >= 80:   
                self.yeni_tab = None
                self.tabela_gecmisi.clear()
                self.gecis('lane_following')

        elif self.state == 'girilmez_sol':
            self.send_control_command(-20, ileri_geri=50, vites=0)
            if self.state_counter >= 50:   
                self.gecis('girilmez_serit')

        elif self.state == 'girilmez_serit':
            if self.current_angle is not None:
                self.send_control_command(self.current_angle, ileri_geri=50, vites=0)
            if self.state_counter >= 40:   
                self.yeni_tab = None
                self.tabela_gecmisi.clear()
                self.gecis('lane_following')

        elif self.state == 'sag_don':
            self.send_control_command(20, ileri_geri=50, vites=0)
            if self.state_counter >= 50:   
                self.gecis('sag_serit')

        elif self.state == 'sag_serit':
            if self.current_angle is not None:
                self.send_control_command(self.current_angle, ileri_geri=50, vites=0)
            if self.state_counter >= 40:   
                self.yeni_tab = None
                self.tabela_gecmisi.clear()
                self.gecis('lane_following')

        elif self.state == 'park_bekle':
            if self.current_tabela == "sol_yasak" or self.state_counter >= 100:
                self.yeni_tab = None
                self.tabela_gecmisi.clear()
                self.gecis('lane_following')
            else:
                self.send_control_command(0, ileri_geri=0, vites=0)

        elif self.state == 'lane_following':
            if self.yeni_tab == "durak":
                self.gecis('durak_yaklas')
            elif self.yeni_tab == "dur":
                self.get_logger().info('DUR tabelası - 8 saniye duruyorum')
                self.gecis('dur_bekle')
            elif self.yeni_tab == "girilmez" and self.solRedCount >= 10:
                self.get_logger().info('GİRİLMEZ - Sola dönüyorum')
                self.gecis('girilmez_sol')
            elif self.yeni_tab == "sag":
                self.get_logger().info('SAĞ tabelası - Sağa dönüyorum')
                self.gecis('sag_don')
            elif self.current_tabela == "park_yasakiki":
                self.get_logger().info('PARK YASAK - Bekliyorum')
                self.gecis('park_bekle')
            else:
                if self.current_angle is not None:
                    self.send_control_command(self.current_angle, ileri_geri=50, vites=0)
                else:
                    self.get_logger().debug('Açı verisi yok, bekliyorum')

    def destroy_node(self):
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
