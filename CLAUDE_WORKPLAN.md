# ISA_TEAM_ANAKOD — ROS2 Otonom Araç Platformu

ROS2 Humble tabanlı otonom araç sistemi. Kamera + Lidar + IMU verilerini işleyerek şerit takibi, nesne tespiti ve donanım kontrolü sağlar.

Stack: Python 3.8+, ROS2 Humble, OpenCV, YOLO/Ultralytics, pyserial, Docker

---

## Donanım Mimarisi

```
[Arduino /dev/ttyUSB0 @115200]   [Arduino /dev/ttyACM1 @9600]
         |                                  |
   encoder_node                       control_node
  (mesafe + hız)                      (IMU açısı)
         |                                  |
         v                                  v
    /encoder/distance              /imu/angle (Float32)
    /encoder/speed
                   \                    /
                    --> control_node <--
                    (+ /lane/angle, /lidar/obstacle, /detection/objects)
                         |
                         v
                    teensy_node
                         |
                [Teensy /dev/ttyACM0 @9600]
                  CSV: sag_sol,ileri_geri,vites,otonom,pc_aci\n
                  → araç hareketi (direksiyon + hız + vites)
```

### Arduino → Teensy Seri Haberleşme Detayı

**Arduino bağlantıları:**
| Port | Baudrate | Okunan veri | Node |
|------|----------|-------------|------|
| `/dev/ttyUSB0` | 115200 | Encoder mesafe (int, mm) | `encoder_node` |
| `/dev/ttyACM1` | 9600 | IMU açısı (float, derece) | `control_node` |

**Teensy komut paketi (CSV, `\n` sonlandırmalı):**
```
sag_sol,ileri_geri,vites,otonom,pc_aci\n
```
| Alan | Tür | Aralık | Açıklama |
|------|-----|--------|----------|
| `sag_sol` | int | -32..+32 | Direksiyon açısı (−=sol, +=sağ) |
| `ileri_geri` | int | 0..100 | Hız (0=dur, 50=normal) |
| `vites` | int | 0..N | Vites kademesi |
| `otonom` | int | 0 / 1 | 0=manuel, 1=otonom |
| `pc_aci` | int | — | PC'den gelen açı verisi |

**Teensy port:** `/dev/ttyACM0` @ 9600 baud, 50 Hz gönderim (timer: 0.02s)

> **Bilinen sorun:** Arduino→Teensy paket format tutarsızlığı, `pc_aci` alanı henüz güncellenmemekte (sabit 0). `teensy_node.py` içinde TODO işaretli.
---

## ROS2 Topic Haritası

### Sensörler (Publish)
| Topic | Mesaj Tipi | Kaynak Node |
|-------|-----------|-------------|
| `/encoder/distance` | `Int32` | `encoder_node` |
| `/encoder/speed` | `Float32` | `encoder_node` |
| `/camera/image` | `Image` | `camera_node` |
| `/lidar/scan` | `LaserScan` | `lidar_node` |
| `/lidar/obstacle` | `Int32` (0/1/2) | `lidar_node` |
| `/imu/angle` | `Float32` | `control_node` |

### Algılama (Publish)
| Topic | Mesaj Tipi | Kaynak Node |
|-------|-----------|-------------|
| `/lane/angle` | `Float32` | `lane_detection_node` |
| `/lane/offset` | `Float32` | `lane_detection_node` |
| `/detection/objects` | `String` | `object_detection_node` |
| `/detection/distance` | `Float32` | `object_detection_node` |

### Kontrol (Publish)
| Topic | Mesaj Tipi | Kaynak Node |
|-------|-----------|-------------|
| `/control/sag_sol` | `Int32` | `control_node` |
| `/control/ileri_geri` | `Int32` | `control_node` |
| `/control/vites` | `Int32` | `control_node` |
| `/algorithm/current` | `String` | `control_node` |

### Joystick / Manuel Mod
| Topic | Mesaj Tipi |
|-------|-----------|
| `/joystick/ileri_geri` | `Int32` |
| `/joystick/sag_sol` | `Int32` |
| `/joystick/vites` | `Int32` |
| `/joystick/manual_mode` | `Bool` |

---

## Kontrol State Machine (control_node)

Durum geçişleri `control_loop` (10 Hz) içinde işlenir. Hiçbir state `time.sleep` kullanmaz.

```
lane_following
  ├─ engel_durumu==2  → DUR (seri komut: ileri_geri=0)
  ├─ engel_durumu==1  → engel_kacin_sol → engel_kacin_sag → engel_kacin_ileri → lane_following
  ├─ tabela=="durak"  → durak_yaklas → durak_sag → durak_serit → durak_dur(300t) → durak_sol → lane_following
  ├─ tabela=="dur"    → dur_bekle(80t) → lane_following
  ├─ tabela=="girilmez" → girilmez_sol → girilmez_serit → lane_following
  ├─ tabela=="sag"    → sag_don → sag_serit → lane_following
  └─ tabela=="park_yasakiki" → park_bekle → lane_following
```

Tabela onayı: aynı tabela **5 ardışık frame**'de görülmeli (`tabela_gecmisi` listesi).

---

## Node'lar

| Node | Dosya | Görev |
|------|-------|-------|
| `camera_node` | `sensors/camera_node.py` | Kamera görüntüsü yayını |
| `encoder_node` | `sensors/encoder_node.py` | Arduino'dan encoder okuma, mesafe+hız publish |
| `lidar_node` | `sensors/lidar_node.py` | Lidar scan + DBSCAN engel tespiti |
| `joystick_node` | `sensors/joystick_node.py` | Joystick → ROS2 topic |
| `control_node` | `control/control_node.py` | State machine, Arduino IMU okuma, kontrol publish |
| `teensy_node` | `control/teensy_node.py` | CSV paket oluştur → Teensy seri gönder |
| `lane_detection_node` | `perception/lane_detection_node.py` | Şerit açı/offset hesaplama |
| `object_detection_node` | `perception/object_detection_node.py` | YOLO tabela/nesne tespiti |
| `gui_node` | `gui/gui_node.py` | PyQt izleme arayüzü (KRİTİK: thread sorunu) |

---

## Kurulum

### Yerel
```bash
# Gereksinimler: ROS2 Humble, Python 3.8+
pip install pyserial ultralytics torch torchvision "numpy<2"

cd ros2_ws
colcon build
source install/setup.bash
```

### Docker
```bash
docker compose build
docker compose run --rm ros2
# Container içinde:
source /opt/ros/humble/setup.bash
cd /workspace/ros2_ws && colcon build && source install/setup.bash
```

`docker-compose.yml` → `/dev/ttyACM0` map edilmiş (Teensy). Farklı port için güncelle.

---

## Çalıştırma

```bash
source ros2_ws/install/setup.bash

# Tüm sistem (launch):
ros2 launch otonom_arac otonom_arac_launch.py

# Tekil node:
ros2 run otonom_arac encoder_node
ros2 run otonom_arac teensy_node
ros2 run otonom_arac control_node
ros2 run otonom_arac lane_detection_node
ros2 run otonom_arac gui_node
```

Port parametresi override:
```bash
ros2 run otonom_arac encoder_node --ros-args -p port:=/dev/ttyUSB1 -p baudrate:=115200
ros2 run otonom_arac teensy_node --ros-args -p teensy_port:=/dev/ttyACM0 -p teensy_baudrate:=9600
```

---

## Sorun Giderme

**"Port bulunamadı" (`/dev/ttyACM0`, `/dev/ttyUSB0`):**
```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
# Yetersiz izin varsa:
sudo chmod 666 /dev/ttyACM0
```

**Arduino veri gelmiyor (`encoder_node`):**
- Baudrate kontrolü: Arduino kodu 115200 olmalı
- `encoder_node` `"1000"` komutu gönderiyor, Arduino integer döndürmeli

**Teensy komutu gitmiyor:**
- `teensy_node` 9600 baud bekliyor, Teensy tarafı aynı olmalı
- `docker-compose.yml` device map'i kontrol et

**YOLO / numpy hatası:**
- `numpy<2` şartı zorunlu: `pip install "numpy<2"`

**GUI thread hatası (`gui_node`):**
- PyQt event loop ile ROS2 spin çakışması mevcut — KRİTİK sorun, `CLAUDE_WORKPLAN.md` bak

---

## Test
```bash
cd ros2_ws
colcon test
colcon test-result --verbose
```
Testler: `ament_flake8`, `ament_pep257`, `pytest`

---

## Repo Yapısı
```
ros2_ws/src/otonom_arac/
  otonom_arac/nodes/
    control/    control_node.py  teensy_node.py
    gui/        gui_node.py
    perception/ lane_detection_node.py  object_detection_node.py
    sensors/    camera_node.py  encoder_node.py  joystick_node.py  lidar_node.py
    testing/    video_player_node.py
  launch/       otonom_arac_launch.py
  test/         test_flake8.py  test_pep257.py  test_copyright.py
Dockerfile
docker-compose.yml
CLAUDE_WORKPLAN.md   ← Claude kod inceleme talimatları
```

Maintainer: `faikaktss`
README.md

copilot/create-structured-readme-research
Send message