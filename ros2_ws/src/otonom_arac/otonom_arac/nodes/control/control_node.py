#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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

        self.declare_parameter('arduino_port', '/dev/ttyUSB11')
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
        # lidar_node'dan gelen hazır engel durumu (0/1/2)
        self.lidar_obstacle_sub = self.create_subscription(
            Int32, '/lidar/obstacle', self.lidar_obstacle_callback, 10)

        self.sag_sol_pub = self.create_publisher(Int32, '/control/sag_sol', 10)
        self.ileri_geri_pub = self.create_publisher(Int32, '/control/ileri_geri', 10)
        self.vites_pub = self.create_publisher(Int32, '/control/vites', 10)
        # GUI için mevcut state adını yayınla
        self.algorithm_pub = self.create_publisher(String, '/algorithm/current', 10)

        self.imu_pub = self.create_publisher(Float32, '/imu/angle', 10)
        self.imu_angle = 0.0

        self.current_angle = None
        self.current_offset = None
        self.current_tabela = None
        self.tabela_distance = 0.0
        self.engel_durumu = 0
        self._obstacle_topic_aktif = False  # /lidar/obstacle geldi mi?

        self.tabela_gecmisi = []
        self.yeni_tab = None
        self.durakPixel = 0

        self.solRedCount = 0
        self.sagRedCount = 0

        self.state = 'lane_following'
        self.state_counter = 0

        self._hareketli_engel_count = 0
        self._logged_hareketli_engel = False
        self._last_logged_durak_pixel = -1
        self._durak_log_count = 0

        # TERMINAL: veri zaman takibi (timeout tespiti için)
        self._imu_last_time      = 0.0
        self._lane_last_time     = 0.0
        self._detection_last_time= 0.0
        self._lidar_last_time    = 0.0
        self._state_start_time   = time.time()

        self.arduino = None
        if SERIAL_AVAILABLE:
            try:
                self.arduino = serial.Serial(arduino_port, arduino_baudrate, timeout=0.01)
                self.get_logger().info(f'Arduino bağlandı: {arduino_port} @ {arduino_baudrate}')
            except Exception as e:
                self.get_logger().warn(f'Arduino bağlantı hatası ({arduino_port}): {e}')
                self.get_logger().warn('Control node Arduino olmadan devam ediyor')
        else:
            self.get_logger().error('pyserial bulunamadı, arduino bağlantısı kurulamıyor')

        self.control_timer = self.create_timer(0.1, self.control_loop)
        # TERMINAL: 1 saniyede bir durum özeti
        self._status_1s_timer = self.create_timer(1.0, self._terminal_status_1s)
        # TERMINAL: 5 saniyede bir sistem sağlık özeti
        self._health_5s_timer = self.create_timer(5.0, self._terminal_health_5s)

        # IMU timer kendi thread'inde çalışır — serial readline() control_loop'u bloklamamaz
        self._imu_cb_group = MutuallyExclusiveCallbackGroup()
        self.imu_timer = self.create_timer(0.02, self.imu_okuma,
                                           callback_group=self._imu_cb_group)

        self.get_logger().info('Control node başlatıldı')
        # TERMINAL: başlangıç özeti
        arduino_str = 'bağlı' if self.arduino else 'YOK'
        print(f'[CONTROL] Başlatıldı | Arduino: {arduino_str} | Port: {arduino_port}', flush=True)

    def imu_okuma(self):
        """Arduino'dan IMU açısını oku ve /imu/angle topic'ine yayınla"""
        if self.arduino is None:
            return
        try:
            # Maksimum 5 satır oku — sonsuz döngü riski önlenir
            for _ in range(5):
                if self.arduino.in_waiting == 0:
                    break
                line = self.arduino.readline().decode(errors='ignore').strip()
                if line:
                    try:
                        self.imu_angle = float(line)
                        # TERMINAL: IMU zaman güncelle
                        self._imu_last_time = time.time()
                        imu_msg = Float32()
                        imu_msg.data = self.imu_angle
                        self.imu_pub.publish(imu_msg)
                    except ValueError:
                        pass
        except Exception as e:
            self.get_logger().warning(f'IMU okuma hatası: {e}')

    def lane_angle_callback(self, msg):
        self.current_angle = int(msg.data)
        # TERMINAL: lane zaman güncelle
        self._lane_last_time = time.time()

    def lane_offset_callback(self, msg):
        self.current_offset = msg.data

    def detection_callback(self, msg):
        self.current_tabela = msg.data if msg.data != "None" else None
        # TERMINAL: detection zaman güncelle
        self._detection_last_time = time.time()
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

    def lidar_obstacle_callback(self, msg):
        """lidar_node'dan gelen hazir engel durumu: 0=yok, 1=durugan, 2=hareketli/kritik"""
        self._obstacle_topic_aktif = True
        self.engel_durumu = msg.data
        # TERMINAL: lidar zaman güncelle
        self._lidar_last_time = time.time()

    def lidar_callback(self, msg):
        """Yedek: lidar_node /lidar/obstacle yayinlamazsa ham scan'den hesapla."""
        if self._obstacle_topic_aktif:
            return  # lidar_node zaten DBSCAN sonucunu gönderiyor, ham hesaba gerek yok
        ranges = msg.ranges
        if len(ranges) == 0:
            return
        front_indices = list(range(len(ranges) // 2 - 15, len(ranges) // 2 + 15))
        front_ranges = [ranges[i] for i in front_indices if i < len(ranges)]
        min_distance = min([r for r in front_ranges if r > 0.0], default=10.0)
        if min_distance < 0.3:
            self.engel_durumu = 2
        elif min_distance < 0.5:
            self.engel_durumu = 1
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
        """State gecisi - sayaci sifirla ve GUI'ye yayinla"""
        self.get_logger().info(f'State: {self.state} -> {yeni_state}')
        self.state = yeni_state
        self.state_counter = 0
        # TERMINAL: state süresi takibi
        self._state_start_time = time.time()
        alg_msg = String()
        alg_msg.data = yeni_state
        self.algorithm_pub.publish(alg_msg)

    def control_loop(self):
        """Her 0.1 saniyede bir çalışır - sadece 1 adım yapar, ASLA uyumaz"""

        self.state_counter += 1

        if self.state == 'lane_following':
            if self.engel_durumu == 2:
                # Log azaltma: İlk kez logla, bitince kaç kez tekrar ettiğini göster
                if not self._logged_hareketli_engel:
                    self.get_logger().info('Hareketli engel - DUR!')
                    self._logged_hareketli_engel = True
                self._hareketli_engel_count += 1
                self.send_control_command(0, ileri_geri=0, vites=0)
                return
            else:
                # Engel kalktı, kaç kez tekrar ettiğini logla
                if hasattr(self, '_logged_hareketli_engel') and self._logged_hareketli_engel:
                    if self._hareketli_engel_count > 1:
                        self.get_logger().info(f'Hareketli engel bitti ({self._hareketli_engel_count}x döngü)')
                    self._logged_hareketli_engel = False
                    self._hareketli_engel_count = 0
            if self.engel_durumu == 1:
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
                # Durak bekleme bitti, özet log
                if hasattr(self, '_durak_log_count') and self._durak_log_count > 0:
                    self.get_logger().info(f'Durak yaklaşma tamamlandı ({self._durak_log_count} güncelleme)')
                    self._durak_log_count = 0
                self.gecis('durak_sag')
            else:
                if self.current_angle is not None:
                    self.send_control_command(self.current_angle, ileri_geri=50, vites=0)
                # Log azaltma: Sadece önemli değişimlerde log, sonunda özet
                if abs(self.durakPixel - self._last_logged_durak_pixel) > 100:  # 100 piksel fark varsa log
                    self.get_logger().info(f'Durak bekleniyor: {self.durakPixel}')
                    self._last_logged_durak_pixel = self.durakPixel
                    self._durak_log_count += 1

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
            elif self.yeni_tab == "park_yasakiki":
                self.get_logger().info('PARK YASAK - Bekliyorum')
                self.gecis('park_bekle')
            else:
                if self.current_angle is not None:
                    angle = self.current_angle
                    if angle < 0:
                        angle -= 4
                    elif angle > 0:
                        angle += 3
                    self.send_control_command(angle, ileri_geri=50, vites=0)
                else:
                    self.get_logger().debug('Aci verisi yok, bekliyorum')

    # TERMINAL: 1 saniyede bir IMU + state özeti
    def _terminal_status_1s(self):
        _now = time.time()
        engel_str = ['YOK', 'SABİT', 'HAREKETLİ'][min(self.engel_durumu, 2)]
        angle_str = str(self.current_angle) if self.current_angle is not None else '-'
        offset_str = f'{self.current_offset:.1f}' if self.current_offset is not None else '-'
        print(
            f'[CONTROL] IMU={self.imu_angle:.1f}° | state={self.state} | '
            f'engel={engel_str} | açı={angle_str} | offset={offset_str}',
            flush=True)
        # IMU timeout uyarısı
        if self._imu_last_time > 0 and (_now - self._imu_last_time) > 3.0:
            print(f'[CONTROL] ⚠ IMU verisi gelmiyor! ({_now - self._imu_last_time:.0f}s süredir)', flush=True)
        # Lane timeout uyarısı
        if self._lane_last_time > 0 and (_now - self._lane_last_time) > 3.0:
            print(f'[CONTROL] ⚠ Lane açısı gelmiyor! Kör gidiyorum.', flush=True)

    # TERMINAL: 5 saniyede bir sistem sağlık özeti
    def _terminal_health_5s(self):
        _now = time.time()
        def _durum(last_t, val_str):
            if last_t == 0.0:
                return f'✗ VERİ YOK  '
            ago = _now - last_t
            ok = ago <= 3.0
            return f'{"✓" if ok else "✗"} {val_str:<12}| {ago:.0f}s önce'
        imu_d    = _durum(self._imu_last_time,       f'{self.imu_angle:.1f}°')
        lane_d   = _durum(self._lane_last_time,       f'açı={self.current_angle or "-"}')
        det_d    = _durum(self._detection_last_time,  str(self.current_tabela or '-'))
        lidar_d  = _durum(self._lidar_last_time,      f'engel={self.engel_durumu}')
        state_s  = _now - self._state_start_time
        print('\n━━ SİSTEM ÖZET [5s] ' + '━'*36, flush=True)
        print(f'[SYS] IMU       : {imu_d}', flush=True)
        print(f'[SYS] Lane      : {lane_d}', flush=True)
        print(f'[SYS] Detection : {det_d}', flush=True)
        print(f'[SYS] LiDAR     : {lidar_d}', flush=True)
        print(f'[SYS] State     : {self.state:<20}| {state_s:.0f}s süredir', flush=True)
        print('━'*54, flush=True)

    def destroy_node(self):
        if self.arduino:
            self.arduino.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    # MultiThreadedExecutor: imu_timer (serial blocking) ve control_loop ayrı thread'lerde
    # çalışır — biri diğerini bloklayamaz → control jitter ortadan kalkar
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
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