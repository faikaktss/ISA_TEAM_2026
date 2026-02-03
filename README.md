# ISA_TEAM_2025-2026_ROS2

Bu proje, otonom araç sistemlerinin sensör verilerini (kamera, Lidar, IMU, GPS), bilgisayarla görü (CV), nesne tanıma (YOLO), şerit takibi ve karar mekanizmaları ile beraber donanım kontrolünü gerçekleştiren bir ROS2 tabanlı platform geliştirmektedir.

## İçerik

- [Proje Amacı ve Özeti](#proje-amacı-ve-özeti)
- [Sistem Gereksinimleri ve Kurulum](#sistem-gereksinimleri-ve-kurulum)
- [Bağımlılıklar](#bağımlılıklar)
- [Donanım Gereksinimleri](#donanım-gereksinimleri)
- [Proje Yapısı](#proje-yapısı)
- [Çalıştırma Talimatları](#çalıştırma-talimatları)
- [Ana Özellikler](#ana-özellikler)
- [Katkı Sağlayanlar ve Lisans](#katkı-sağlayanlar-ve-lisans)

---

## Proje Amacı ve Özeti

Bu repodaki yazılım; ROS2 çatısı üzerinde, gerçek sensörlerden (LIDAR, ZED/Logitech/Intel Realsense kameralar, Teensy ve Arduino ile motor-servo mevzi kontrolü, IMU, GPS vb.) alınan verilerin işlenmesi, yol-şerit algılama, trafik levhası tanıma, nesne tespiti ve alınan karar doğrultusunda donanımın akıllı biçimde yönetilmesi amacıyla geliştirilmiştir.

Öne çıkan yetenekler:
- Derin öğrenme tabanlı nesne ve tabela (levha) tanıma (YOLO ile).
- Şerit algılama, yol pozisyonu ve açı hesabı.
- Lidar ile engel tespiti, mesafe ölçümü ve yol üzerinde dinamik karar.
- Grafiksel kullanıcı arayüzü (PyQt5 / Qt).
- ROS2 ile farklı modüllerden veri akışı ve komut yönetimi.

---

## Sistem Gereksinimleri ve Kurulum

1. **Python 3.8 veya üstü**
2. **ROS2 Foxy/Foxy tabanlı bir ROS2 kurulumu**
3. **Resmi ve ek bağımlılıklar:**
    - `opencv-python`
    - `numpy`
    - `PyQt5`
    - `ultralytics` (YOLOv8 üzerinde çalışmak için)
    - `pyzed` (**ZED kamera için**)
    - `pyrealsense2` (**Intel Realsense için**)
    - `rplidar` (**RP-Lidar için**)
    - `setuptools`
    - (Varsa) Arduino/Teensy Python kütüphaneleri

Kurulumu örnek:

```bash
# Gerekli bağımlılıkları yükleyin:
pip install opencv-python numpy PyQt5 ultralytics pyzbar pyrealsense2 rplidar

# ROS2 ortamını etkinleştirin
source /opt/ros/foxy/setup.bash

# Workspace dizininde ROS2 paketini kurun:
cd ros2_ws
colcon build
source install/setup.bash
```
> Not: Kameralara/hardware'e özgü driver ve SDK'lar için ilgili üreticinin sitesinden dökümana bakmanız gerekebilir.

---

## Donanım Gereksinimleri

- Logitech veya Realsense veya ZED kamera (kamera tespiti ve şerit takibi için)
- RP-Lidar (engel algılama ve mesafe için)
- Arduino & Teensy kartları (motor/servo ve ek donanımlar kontrolü için)
- Robot platformu ve temel güç birimi

---

## Proje Yapısı

```
ISA_TEAM_2025-2026_ROS2/
│
├── main.py                   # Ana uygulama - tüm modüllerin başlangıç noktası
├── LaneDetect.py             # Şerit algılama ve açı hesaplama
├── lidar.py                  # Lidar verisi işleyici
├── camera.py                 # Kamera portu için wrapper
├── info.py                   # Robotun anlık durum bilgileri
├── arduino.py                # Arduino/Teensy donanımı ile haberleşme
├── qt_Arayüz/                # PyQt tabanlı grafik arayüz dosyaları
├── modeller/                 # YOLO modelleri
├── yardımcıDosyalar/         # Yardımcı ve test scriptleri (örn. nokta_belirle.py ile ZED kalibrasyonu)
├── ros2_ws/                  # ROS2 workspace'i (modüller, map vb.)
└── README.md                 # Bu dosya (dökümantasyon)
```

---

## Ana Özellikler

### 1. Şerit ve Yol Algılama
Gelen kamera görüntüsü üzerinden şeritlerin konumu ve açı hesabı yapılır. `LaneDetect.py` ana modülüdür.

### 2. Trafik Levhası ve Nesne Tanıma
Nesne ve trafik levhası tespiti, YOLO modeliyle yapılır. Modeller `/modeller/best.pt` gibi kendi eğittiğiniz veya hazır bir modele referans veriyor.

### 3. LIDAR ile Engel Algılama
`lidar.py` ve donanım bağlantısı ile çevre haritası çıkarılır, engeller algılanır ve GUI'de gösterilir.

### 4. Donanım ve Karar Mekanizması
- Arduino ve Teensy üzerinden hız-açı yönetimi ve servo kontrol sağlanır.
- Entegre edilen komut sınıfları ile, tabela ve engel durumuna göre dinamik kararlar alınır.

### 5. Grafiksel Arayüz (GUI)
Kullanıcıya; kameradan gelen görüntüyü, şeritleri, nesneleri, algılanan levhaları ve haritayı bir arada sunan bir GUI sağlanır.

---

## Çalıştırma Talimatları

**Basit simülasyon:** (Kamera varsa)

```bash
python main.py
```

**Alternatif & test için:**  
`yardımcıDosyalar/main_logitech.py` veya çalışma ortamınıza uygun diğer dosyaları çalıştırabilirsiniz.

---

## Örnek Kullanım Senaryosu

1. Arduino ve Teensy kartlarını ve sensörleri PC'ye/bilgisayara bağlayın.
2. Kamerayı ve Lidar'ı uygun porta takın, donanımların PC tarafından tanındığından emin olun.
3. ROS2 ortamınızı aktive edin.
4. `main.py`'yi veya ilgili yardımcı `main_*.py` dosyalarını çalıştırın.
5. GUI üzerinden verileri, anlık yol durumunu ve aldığı kararları gözlemleyin.

---


**Notlar:**
- Donanımınız ve konfigürasyonunuza göre port ayarlarını, model yollarını ve sistem parametrelerinizi ilgili scriptlerde değiştiriniz.
- YOLO ağırlıkları, harici modeller ve daha fazla bilgi için `modeller/` klasörünü inceleyiniz.
- ROS2 paketlerini ve düğümlerini (nodes) kendi uygulama senaryonuza göre yeniden isimlendirin ya da uyarlayın.
