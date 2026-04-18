
 <!-- ========================= -->
<!--  ISA TEAM ROS2 README     -->
<!-- ========================= -->

<div align="center">

# 🚗 ISA_TEAM_ANAKOD

**ROS2 tabanlı otonom araç algoritma ve donanım kontrol platformu**  
Kamera + Lidar + (opsiyonel) IMU/GPS verilerini işleyerek algılama/karar/control akışını yönetir.

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python)
![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Foxy-blueviolet?logo=ros)
![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20Windows-orange?logo=linux)
![Docker](https://img.shields.io/badge/Docker-Supported-2496ED?logo=docker)

</div>

---

## 📌 Proje Özeti 

Bu repo; **otonom araç** için geliştirilen bir ROS2 uygulamasının çalışma alanını (`ros2_ws`) ve bileşenlerini içerir. Ana hedef:

- 🧠 **Algılama (Perception):** Şerit/yol algılama, nesne tespiti (YOLO/Ultralytics)
- 🛑 **Sensörler:** Kamera, lidar, encoder, joystick gibi kaynaklardan veri toplama
- 🎛️ **Kontrol:** Kontrol kararlarının Arduino/Teensy gibi gömülü kartlara iletilmesi (seri haberleşme)
- 🖥️ **GUI:** PyQt tabanlı izleme/arayüz (ROS2 node olarak çalışır)

---

## 🧭 İçindekiler

- [✨ Öne Çıkan Özellikler](#-öne-çıkan-özellikler)
- [📦 Repo Yapısı](#-repo-yapısı)
- [🧱 ROS2 Paketleri ve Node’lar](#-ros2-paketleri-ve-nodelar)
- [⚙️ Kurulum (Yerel)](#️-kurulum-yerel)
- [🐳 Docker ile Çalıştırma](#-docker-ile-çalıştırma)
- [🚀 Çalıştırma (ROS2)](#-çalıştırma-ros2)
- [🧩 Mimari / Veri Akışı](#-mimari--veri-akışı)
- [🛠️ Test / Kod Kalitesi](#️-test--kod-kalitesi)
- [🧯 Sorun Giderme](#-sorun-giderme)
- [🤝 Katkı & Lisans](#-katkı--lisans)

---

## ✨ Öne Çıkan Özellikler

| Başlık | Açıklama |
|---|---|
| 🏁 Perception | Şerit algılama + nesne tespiti (YOLO/Ultralytics) |
| 🛑 Lidar | Lidar verisini okuyup işleme (paket içinde `lidar_node`) |
| 🎮 Teleop | Joystick ile kontrol / test (`joystick_node`) |
| 🧾 Encoder | Encoder verisi okuma (`encoder_node`) |
| 🔌 Donanım Haberleşmesi | Teensy/Arduino ile seri haberleşme (`teensy_node`) |
| 🖥️ GUI | ROS2 GUI node’u (`gui_node`) |

---

## 📦 Repo Yapısı

Repo kökünde önemli dosyalar:

- `README.md` → bu doküman  
- `Dockerfile` → ROS2 Humble desktop tabanlı docker imajı
- `docker-compose.yml` → cihaz bağlama (`/dev/ttyACM0`) + volume + host network
- `ros2_ws/` → ROS2 workspace (src/build/install/log içeriyor)

ROS2 workspace içinde:

- `ros2_ws/src/otonom_arac/` → ana ROS2 paketi (ament_python)

> Not: `ros2_ws/build/` ve `ros2_ws/install/` klasörleri repoda görünüyor. Normalde build çıktıları repoya commit edilmez; ama burada mevcut olduğu için hem kaynak hem çıktılar yan yana duruyor.

---

## 🧱 ROS2 Paketleri ve Node’lar

### Paket: `otonom_arac`

Paket bilgisi:
- Paket adı: `otonom_arac`
- Build sistemi: `ament_python`
- Bağımlılıklar: `rclpy`, `std_msgs`, `sensor_msgs`, `cv_bridge`

📍 Paket yolu:
- `ros2_ws/src/otonom_arac/`

### Console scripts (çalıştırılabilir node komutları)

Aşağıdaki node’lar paketin `setup.py` içindeki `console_scripts` kısmından geliyor:

#### Sensör Node’ları
- `camera_node` → `otonom_arac.nodes.sensors.camera_node:main`
- `encoder_node` → `otonom_arac.nodes.sensors.encoder_node:main`
- `lidar_node` → `otonom_arac.nodes.sensors.lidar_node:main`
- `joystick_node` → `otonom_arac.nodes.sensors.joystick_node:main`

#### Kontrol Node’ları
- `control_node` → `otonom_arac.nodes.control.control_node:main`
- `teensy_node` → `otonom_arac.nodes.control.teensy_node:main`

#### Algılama (Perception) Node’ları
- `lane_detection_node` → `otonom_arac.nodes.perception.lane_detection_node:main`
- `object_detection_node` → `otonom_arac.nodes.perception.object_detection_node:main`

#### GUI
- `gui_node` → `otonom_arac.nodes.gui.gui_node:main`

---

## ⚙️ Kurulum (Yerel)

### 1) Sistem gereksinimleri

- ROS2: **Humble** önerilir (Docker imajı Humble kullanıyor)
- Python: 3.8+
- OpenCV, Numpy
- YOLO/Ultralytics (nesne tespiti için)
- Seri haberleşme için: `pyserial`

> Repo içindeki Dockerfile ayrıca şu paketleri kuruyor: `ultralytics`, `pyserial`, `torch/torchvision (CPU)` ve `numpy<2`.

### 2) ROS2 workspace build

```bash
cd ros2_ws
# (opsiyonel) rosdep ile bağımlılık çözmek istersen:
# rosdep install --from-paths src -i -y

colcon build
source install/setup.bash
```

---

## 🐳 Docker ile Çalıştırma

Bu repo Docker ile kolay başlatma için hazırlanmış:

### Dockerfile (özet)
- Base image: `osrf/ros:humble-desktop`
- Kurulanlar: `python3-pip`, `python3-opencv`, `numpy<2`
- Kurulan Python paketleri: `torch/torchvision (CPU)`, `ultralytics`, `pyserial`

### docker-compose (özet)
- `./ros2_ws` → container içinde `/workspace/ros2_ws` olarak mount edilir
- Seri port map: `/dev/ttyACM0:/dev/ttyACM0` (Teensy/Arduino için)
- `network_mode: host` (ROS2 keşfi ve network için pratik)

Çalıştırma:

```bash
docker compose build
docker compose run --rm ros2
```

Container içinde:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/ros2_ws
colcon build
source install/setup.bash
```

> Donanım portu farklıysa `docker-compose.yml` içindeki `/dev/ttyACM0` satırını kendi cihazına göre güncelle.

---

## 🚀 Çalıştırma (ROS2)

> Aşağıdaki komutlar için önce workspace’i source etmeyi unutma:
```bash
cd ros2_ws
source install/setup.bash
```

### Tek tek node çalıştırma

Örnekler:

```bash
ros2 run otonom_arac camera_node
ros2 run otonom_arac lidar_node
ros2 run otonom_arac lane_detection_node
ros2 run otonom_arac object_detection_node
ros2 run otonom_arac teensy_node
ros2 run otonom_arac gui_node
```

### Launch ile çalıştırma

Repo içinde launch dosyası yolu:
- `ros2_ws/src/otonom_arac/launch/otonom_arac_launch.py`

Çalıştırma:

```bash
ros2 launch otonom_arac otonom_arac_launch.py
```

> Not: Launch dosyasında “tüm gerekli düğümleri başlatacak” yapı TODO olarak işaretli. Kendi node’larını bu launch içine ekleyerek tek komutla bütün sistemi kaldırabilirsin.

---

## 🧩 Mimari / Veri Akışı

Aşağıdaki şema, sistemin genel akışını “yüksek seviyede” anlatır:

```mermaid
flowchart LR
  CAM[📷 camera_node] --> PER1[🛣️ lane_detection_node]
  CAM --> PER2[🎯 object_detection_node]

  LIDAR[🛑 lidar_node] --> CTRL[🧠 control_node]
  PER1 --> CTRL
  PER2 --> CTRL
  ENC[🧾 encoder_node] --> CTRL
  JOY[🎮 joystick_node] --> CTRL

  CTRL --> TEENSY[🔌 teensy_node (serial)]
  CTRL --> GUI[🖥️ gui_node]
  CAM --> GUI
  LIDAR --> GUI
```

---

## 🛠️ Test / Kod Kalitesi

Paket `package.xml` içinde test bağımlılıkları tanımlı:
- `ament_flake8`
- `ament_pep257`
- `python3-pytest`

(Projede test klasörü: `ros2_ws/src/otonom_arac/test/`)

Örnek:

```bash
cd ros2_ws
colcon test
colcon test-result --verbose
```

---

## 🧯 Sorun Giderme

### 1) ROS2 “paketi bulamıyor”
- `source install/setup.bash` yaptığından emin ol.
- `colcon build` hatasız tamamlandı mı kontrol et.

### 2) Seri port hatası (`/dev/ttyACM0` bulunamadı)
- Host’ta cihaz adını kontrol et:
  ```bash
  ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
  ```
- Docker kullanıyorsan `docker-compose.yml` içindeki device mapping’i güncelle.

### 3) YOLO / Ultralytics / Torch problemleri
- Docker yolu daha stabil: CPU torch + ultralytics kurulumu hazır geliyor.
- Yerelde sürüm uyumsuzluğunda `numpy<2` şartını dene.

---

## 🤝 Katkı & Lisans

- Katkı için: Issue/PR açabilir, node’ları modülerleştirip launch dosyasını tamamlayabilirsin.
- Lisans: `package.xml` içinde şu an **TODO** görünüyor. Net bir lisans seçip repo köküne `LICENSE` eklemeni öneririm.

---

## 📬 İletişim / Sorumlular

Maintainer: `faikaktss` (package metadata içinde tanımlı)

---

### ✅ Hızlı Başlangıç (Cheat Sheet)

```bash
# Yerel
cd ros2_ws
colcon build
source install/setup.bash
ros2 run otonom_arac gui_node
```

```bash
# Docker
docker compose build
docker compose run --rm ros2
# container içinde:
source /opt/ros/humble/setup.bash
cd /workspace/ros2_ws
colcon build
source install/setup.bash
ros2 launch otonom_arac otonom_arac_launch.py
```
# ISA-TEAM_ANAKOD
# ISA_TEAM_ANAKOD1
# ISA_TEAM_ANA_KOD1
