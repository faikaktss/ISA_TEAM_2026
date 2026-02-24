# 🚗 GAZEBO SİMÜLASYON ENTEGRASYONU - EKİP ÇALIŞMA PLANI

## 📋 Genel Bakış

Bu doküman, **otonom araç projesinin Gazebo simülasyonuna entegrasyonu** için 3 kişilik ekip çalışma planını içerir. Her kişinin görevi bağımsız olarak çalışabilir, böylece paralel ilerleme sağlanır.

---

## 🎯 Proje Durumu

✅ **Tamamlananlar:**
- ROS2 node'ları hazır (8 adet: camera, encoder, lidar, joystick, lane_detection, object_detection, control, teensy)
- Klasör yapısı oluşturuldu (`urdf/`, `worlds/`, `meshes/`, `config/`)
- Template dosyalar hazırlandı
- `setup.py` güncellendi

⏳ **Yapılacaklar:**
- URDF modelini tamamla (Kişi 1)
- Gazebo dünyasını oluştur (Kişi 2)
- Launch dosyasını düzenle (Kişi 3)
- Linux'ta test et

---

## 👥 Görev Dağılımı

### 🔧 KİŞİ 1: URDF/XACRO MODEL GELİŞTİRİCİSİ

**Dosya:** `ros2_ws/src/otonom_arac/urdf/otonom_arac.urdf.xacro`

**Görevler:**
1. ✅ Chassis (gövde) inertia değerleri **zaten hesaplandı**
2. ✅ 4 tekerlek tanımları **hazır**
3. ✅ Ackermann direksiyon mekanizması **kuruldu**
4. 🔍 **YAPILACAKLAR:**
   - Kamera pozisyonlarını optimize et (satır 331, 359)
   - Lidar yüksekliğini ayarla (satır 388)
   - Gazebo plugin parametrelerini test et (satır 401+)
   - Opsiyonel: Solidworks modelinden STL export edip `meshes/` klasörüne koy

**Test Komutu (Linux):**
```bash
cd ~/ros2_ws
colcon build --packages-select otonom_arac
source install/setup.bash
ros2 launch otonom_arac gazebo_simulation.launch.py
```

**Kontrol Noktaları:**
- [ ] Robot Gazebo'da görünüyor mu?
- [ ] Tekerlekler doğru dönüyor mu?
- [ ] Direksiyon ±45° hareket ediyor mu?
- [ ] Kameralardan görüntü geliyor mu? (`ros2 topic echo /camera/lane/image_raw`)
- [ ] Lidar veri yayınlıyor mu? (`ros2 topic echo /scan`)

**Kaynaklar:**
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)
- [Gazebo ROS Control](https://classic.gazebosim.org/tutorials?tut=ros_control)
- [Ackermann Steering](https://www.youtube.com/watch?v=oYMMdjbmQXc)

---

### 🌍 KİŞİ 2: GAZEBO WORLD TASARIMCISI

**Dosya:** `ros2_ws/src/otonom_arac/worlds/test_track.world`

**Görevler:**
1. ✅ Zemin (ground plane) **oluşturuldu**
2. ✅ Temel şerit çizgileri **mevcut**
3. ✅ Örnek trafik işareti ve koniler **eklendi**
4. 🔍 **YAPILACAKLAR:**
   - Daha fazla şerit çizgisi ekle (kesikli beyaz çizgiler)
   - Viraj bölümü oluştur (box'ları rotate ederek)
   - Trafik işaretleri ekle (Dur, Hız limiti 30, Sağa/Sola dönüş)
   - Park alanı çiz
   - Opsiyonel: Yol texture dosyası hazırla

**Örnek Viraj Kodu:**
```xml
<model name="road_curve">
  <static>true</static>
  <pose>25 5 0.01 0 0 0.785</pose>  <!-- 45° döndürülmüş -->
  <link name="link">
    <visual name="visual">
      <geometry>
        <box>
          <size>10 4 0.01</size>
        </box>
      </geometry>
      <material>
        <ambient>0.2 0.2 0.2 1</ambient>
        <diffuse>0.3 0.3 0.3 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

**Test Komutu (Linux):**
```bash
gazebo --verbose test_track.world
```

**Kontrol Noktaları:**
- [ ] Şerit çizgileri kameradan görünüyor mu?
- [ ] Trafik işaretleri 3-5 metre mesafeden net mi?
- [ ] Viraj açısı gerçekçi mi? (ISA standartları: min 5m yarıçap)
- [ ] Zemin sürtünme katsayısı yeterli mi? (mu1=1.0, mu2=1.0)

**Kaynaklar:**
- [Gazebo World Tutorial](https://classic.gazebosim.org/tutorials?tut=build_world)
- [SDF Format](http://sdformat.org/spec)
- [ISA Yarışma Kuralları](https://www.isaautonomoussystems.com/)

---

### 🚀 KİŞİ 3: LAUNCH DOSYASI & ENTEGRASYON UZMANI

**Dosya:** `ros2_ws/src/otonom_arac/launch/gazebo_simulation.launch.py`

**Görevler:**
1. ✅ Gazebo başlatma komutları **hazır**
2. ✅ Robot spawn node'u **mevcut**
3. ✅ ROS2 node'ları eklendi
4. 🔍 **YAPILACAKLAR:**
   - Topic remapping'i tamamla (satır 241-243)
   - `cmd_vel_bridge` node'u yaz (control_node output → Gazebo input)
   - Serial portları devre dışı bırak (satır 261-262)
   - RViz config dosyasını optimize et
   - Test senaryoları yaz

**Eksik Bridge Node (Yeni dosya gerekli):**

`ros2_ws/src/otonom_arac/otonom_arac/nodes/control/cmd_vel_bridge.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        
        # Subscribers (control_node'dan gelenler)
        self.sub_sag_sol = self.create_subscription(Int32, '/sag_sol', self.sag_sol_callback, 10)
        self.sub_ileri_geri = self.create_subscription(Int32, '/ileri_geri', self.ileri_geri_callback, 10)
        
        # Publisher (Gazebo'ya giden)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.sag_sol_val = 0
        self.ileri_geri_val = 0
        
        self.timer = self.create_timer(0.02, self.publish_cmd_vel)  # 50Hz
    
    def sag_sol_callback(self, msg):
        self.sag_sol_val = msg.data
    
    def ileri_geri_callback(self, msg):
        self.ileri_geri_val = msg.data
    
    def publish_cmd_vel(self):
        twist = Twist()
        
        # Dönüşüm: sag_sol (1450-1750) → angular.z (-1.0 to 1.0)
        # Orta nokta: 1600
        steering_normalized = (self.sag_sol_val - 1600) / 150.0
        steering_normalized = max(-1.0, min(1.0, steering_normalized))
        
        # Dönüşüm: ileri_geri (1300-1800) → linear.x (-2.0 to 2.0)
        # Orta nokta: 1550
        speed_normalized = (self.ileri_geri_val - 1550) / 250.0
        speed_normalized = max(-1.0, min(1.0, speed_normalized))
        
        twist.linear.x = speed_normalized * 2.0  # Max 2 m/s
        twist.angular.z = steering_normalized * 0.785  # Max 45° (0.785 rad)
        
        self.pub_cmd_vel.publish(twist)

def main():
    rclpy.init()
    node = CmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**setup.py'ye ekle:**
```python
'cmd_vel_bridge = otonom_arac.nodes.control.cmd_vel_bridge:main',
```

**Test Komutu (Linux):**
```bash
cd ~/ros2_ws
colcon build --packages-select otonom_arac
source install/setup.bash

# Terminal 1: Gazebo + Robot
ros2 launch otonom_arac gazebo_simulation.launch.py

# Terminal 2: Topic kontrol
ros2 topic list
ros2 topic echo /cmd_vel

# Terminal 3: Manuel kontrol (test için)
ros2 topic pub /sag_sol std_msgs/Int32 "data: 1600"
ros2 topic pub /ileri_geri std_msgs/Int32 "data: 1650"
```

**Kontrol Noktaları:**
- [ ] Gazebo başarıyla açılıyor mu?
- [ ] Robot spawn oluyor mu?
- [ ] Tüm ROS2 node'ları çalışıyor mu? (`ros2 node list`)
- [ ] Topic'ler yayınlanıyor mu? (`ros2 topic list`)
- [ ] /cmd_vel komutuyla robot hareket ediyor mu?
- [ ] RViz'de robot modeli görünüyor mu?

**Kaynaklar:**
- [ROS2 Launch Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [Topic Remapping](https://design.ros2.org/articles/topic_and_service_names.html)
- [Gazebo ROS2 Plugins](https://github.com/ros-simulation/gazebo_ros_pkgs)

---

## 🛠️ Kurulum Gereksinimleri

### Linux (Ubuntu 22.04) - ÖNERİLEN

```bash
# ROS2 Humble kurulumu
sudo apt update
sudo apt install ros-humble-desktop-full

# Gazebo Classic 11
sudo apt install ros-humble-gazebo-ros-pkgs

# Geliştirme araçları
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui

# Opsiyonel: RViz2
sudo apt install ros-humble-rviz2

# Workspace'i derle
cd ~/ros2_ws
colcon build --packages-select otonom_arac
source install/setup.bash
```

### macOS (Sınırlı Destek)

macOS kullanıcıları **Docker** ile çalışmalı:

```bash
# Docker kurulumu
brew install --cask docker

# ROS2 + Gazebo imajı
docker pull osrf/ros:humble-desktop-full

# X11 forwarding (GUI için)
brew install --cask xquartz
open -a XQuartz

# Docker container başlat
docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -v $(pwd):/workspace \
  osrf/ros:humble-desktop-full
```

---

## 📦 Proje Yapısı

```
ros2_ws/src/otonom_arac/
├── nodes/
│   ├── sensors/
│   │   ├── camera_node.py          ✅ Hazır
│   │   ├── lidar_node.py           ✅ Hazır
│   │   └── ...
│   ├── control/
│   │   ├── control_node.py         ✅ Hazır
│   │   └── cmd_vel_bridge.py       ⚠️  Kişi 3 yazacak
│   └── perception/
│       ├── lane_detection_node.py  ✅ Hazır
│       └── object_detection_node.py ✅ Hazır
├── urdf/
│   └── otonom_arac.urdf.xacro      🔨 Kişi 1 optimize edecek
├── worlds/
│   └── test_track.world            🔨 Kişi 2 tamamlayacak
├── meshes/
│   └── (STL/DAE dosyalar)          📁 Opsiyonel
├── config/
│   └── gazebo_view.rviz            ✅ Template hazır
├── launch/
│   ├── otonom_arac_launch.py       ✅ Gerçek araç için
│   └── gazebo_simulation.launch.py 🔨 Kişi 3 tamamlayacak
└── setup.py                        ✅ Güncellendi
```

---

## 🧪 Test Süreci

### Aşama 1: Bireysel Test (Her Kişi Kendi Görevini)

**Kişi 1:**
```bash
# URDF syntax kontrolü
check_urdf otonom_arac.urdf.xacro

# Görselleştirme
ros2 launch urdf_tutorial display.launch.py model:=otonom_arac.urdf.xacro
```

**Kişi 2:**
```bash
# World dosyası testi
gazebo --verbose test_track.world
```

**Kişi 3:**
```bash
# Launch dosyası syntax kontrolü
python3 gazebo_simulation.launch.py --show-args
```

---

### Aşama 2: Entegrasyon Testi (Hep Birlikte)

```bash
# Terminal 1: Gazebo simülasyonu başlat
ros2 launch otonom_arac gazebo_simulation.launch.py

# Terminal 2: Node'ları kontrol et
ros2 node list
# Beklenen output:
# /camera_node
# /lidar_node
# /lane_detection_node
# /object_detection_node
# /control_node
# /cmd_vel_bridge
# /gazebo
# /robot_state_publisher

# Terminal 3: Topic'leri kontrol et
ros2 topic list
# Önemli topic'ler:
# /camera/lane/image_raw
# /camera/sign/image_raw
# /scan
# /cmd_vel
# /lane_angle
# /lane_offset

# Terminal 4: Manuel hareket testi
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.5, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.2}"
```

---

### Aşama 3: Otonom Sürüş Testi

```bash
# Tüm sistemler hazır, control_node devreye girsin
# Araç şerit takibi ve işaret tanıma yapmalı

# Log'ları izle
ros2 topic echo /control/status
ros2 topic echo /tabela_algilama

# Video kaydı al (opsiyonel)
ros2 bag record -a -o test_surush_1
```

---

## 🐛 Yaygın Sorunlar ve Çözümler

### Problem: Gazebo açılmıyor
```bash
# Çözüm 1: Gazebo'yu resetle
killall gzserver gzclient
rm -rf ~/.gazebo/

# Çözüm 2: GPU rendering kapalı
export LIBGL_ALWAYS_SOFTWARE=1
gazebo --verbose
```

### Problem: Robot spawn olmuyor
```bash
# URDF syntax hatası kontrolü
ros2 run xacro xacro otonom_arac.urdf.xacro > temp.urdf
check_urdf temp.urdf

# Gazebo log'larını incele
cat ~/.gazebo/server.log
```

### Problem: Kameradan görüntü gelmiyor
```bash
# Topic'lerin mevcut olup olmadığını kontrol et
ros2 topic hz /camera/lane/image_raw

# Plugin yüklendi mi?
ros2 param list /gazebo

# Manuel test
ros2 run image_tools showimage --ros-args --remap image:=/camera/lane/image_raw
```

### Problem: Robot hareket etmiyor
```bash
# /cmd_vel topic'ine veri geliyor mu?
ros2 topic echo /cmd_vel

# Joint controller'lar aktif mi?
ros2 control list_controllers

# Fizik motoru çalışıyor mu?
gz topic -l  # Gazebo topic'leri
```

---

## 📊 Başarı Kriterleri

### Minimum (MVP):
- [x] Gazebo açılıyor
- [ ] Robot görünüyor ve hareket ediyor
- [ ] Kameralardan görüntü alınıyor
- [ ] Lidar veri yayınlıyor

### İdeal:
- [ ] Otonom şerit takibi çalışıyor
- [ ] Trafik işareti tanıma yapılıyor
- [ ] Control node simülasyonda doğru çalışıyor
- [ ] RViz görselleştirmesi aktif

### Ekstra:
- [ ] Gerçekçi 3D model (STL/DAE mesh)
- [ ] Detaylı yol dokusu (texture)
- [ ] Çoklu test senaryoları
- [ ] Video demo kaydı

---

## 📅 Tahmini Süre

| Kişi | Görev | Süre |
|------|-------|------|
| Kişi 1 | URDF optimize | 2-3 saat |
| Kişi 2 | World oluşturma | 3-4 saat |
| Kişi 3 | Launch + Bridge | 2-3 saat |
| Hepsi | Entegrasyon test | 1-2 saat |
| **TOPLAM** | | **8-12 saat** |

**Paralel çalışmayla:** 1-2 günde tamamlanabilir ✅

---

## 💬 İletişim ve Senkronizasyon

1. **Git branch'ler oluşturun:**
   ```bash
   git checkout -b gazebo-urdf    # Kişi 1
   git checkout -b gazebo-world   # Kişi 2
   git checkout -b gazebo-launch  # Kişi 3
   ```

2. **Her gün sonu merge edin:**
   ```bash
   git pull origin main
   git merge main
   git push origin <branch-name>
   ```

3. **Sorun olursa issue açın:**
   - GitHub Issues kullanın
   - Tag'ler: `gazebo`, `urdf`, `world`, `launch`, `bug`, `help-wanted`

---

## 📚 Ek Kaynaklar

- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [Gazebo Classic Tutorials](https://classic.gazebosim.org/tutorials)
- [URDF/Xacro Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [Ackermann Steering GitHub](https://github.com/ros-drivers/ackermann_msgs)
- [ISA Yarışması](https://www.isaautonomoussystems.com/)

---

## ✅ Checklist (Takip İçin)

### Kişi 1:
- [ ] Kamera pozisyonları optimize edildi
- [ ] Lidar yüksekliği ayarlandı
- [ ] URDF syntax hatası yok
- [ ] Robot Gazebo'da spawn oluyor
- [ ] Tekerlekler dönüyor
- [ ] Sensor topic'leri aktif

### Kişi 2:
- [ ] Şerit çizgileri tamamlandı
- [ ] Viraj eklendi
- [ ] Trafik işaretleri yerleştirildi
- [ ] Park alanı çizildi
- [ ] World syntax hatası yok
- [ ] Gazebo'da yükleniyor

### Kişi 3:
- [ ] `cmd_vel_bridge` node'u yazıldı
- [ ] Topic remapping tamamlandı
- [ ] Serial portlar devre dışı
- [ ] Launch dosyası çalışıyor
- [ ] Tüm node'lar başlıyor
- [ ] RViz config optimize edildi

---

**Başarılar! 🚗💨**

Sorularınız için: GitHub Issues veya ekip iletişim kanalları
