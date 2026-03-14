 # 🚗 ISA_TEAM_2025-2026_ROS2

> **Otonom Araç Algoritma ve Donanım Platformu: Kamera, Lidar, IMU, GPS ile gerçek zamanlı çevresel algı, kontrol ve karar mekanizmaları (ROS2)**

---

## 📑 İçerik
- [🎯 Proje Amacı ve Özeti](#-proje-amacı-ve-özeti)
- [💻 Sistem Gereksinimleri ve Kurulum](#-sistem-gereksinimleri-ve-kurulum)
- [📦 Bağımlılıklar](#-bağımlılıklar)
- [🔌 Donanım Gereksinimleri](#-donanım-gereksinimleri)
- [📂 Proje Yapısı](#-proje-yapısı)
- [🚀 Çalıştırma Talimatları](#-çalıştırma-talimatları)
- [✨ Ana Özellikler](#-ana-özellikler)
- [👤 Katkı Sağlayanlar ve Lisans](#katkı-sağlayanlar-ve-lisans)
---

## 🎯 Proje Amacı ve Özeti
⭐ ROS2 çatısı üzerinde gerçek sensörlerden alınan verilerin işlenmesi, yol-şerit algılama, trafik levhası tanıma, nesne tespiti ve karar ile donanımın yönetilmesi için geliştirilmiştir.

**Öne çıkanlar**  
- 🏁 Derin öğrenme tabanlı nesne & trafik levhası (YOLO)
- 📐 Şerit algılama ve açı hesabı
- 🛑 Lidar ile çevresel engel tespiti
- 🖥️ PyQt5 & Qt arayüz
- 🔄 ROS2 iletişimi

---

## 💻 Sistem Gereksinimleri ve Kurulum

![Python](https://img.shields.io/badge/Python-%3E=3.8-blue?logo=python)
![ROS2 Foxy](https://img.shields.io/badge/ROS2-Foxy-blueviolet?logo=ros)
![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20Windows-orange?logo=linux)  

> Kütüphaneler:
`opencv-python`, `numpy`, `PyQt5`, `ultralytics`, `pyzed` (ZED için), `pyrealsense2`, `rplidar`, `setuptools`, `pyzbar` (isteğe bağlı)

### 🚦 Kurulum

```bash
pip install opencv-python numpy PyQt5 ultralytics pyzbar pyrealsense2 rplidar
source /opt/ros/foxy/setup.bash      # ROS2 aktif etme
cd ros2_ws && colcon build
source install/setup.bash
```

> 💡 Donanımınıza ve platformunuza göre sürücü/SDK kurulumları için üretici dökümanına bakın.

---

## 🔌 Donanım Gereksinimleri

- 📷 Logitech, Realsense veya ZED kamera  
- 🛑 RP-Lidar  
- 🪫 Arduino & Teensy  
- 🤖 Robot platformu, batarya/power supply  

---

## 📂 Proje Yapısı

```
ISA_TEAM_2025-2026_ROS2/
│
├── main.py                # 🚀 Ana başlangıç noktası
├── LaneDetect.py          # 🛣️ Şerit algılama
├── lidar.py               # 🛑 Lidar işleme
├── camera.py              # 📷 Kamera wrapper
├── info.py                # ℹ️ Durum/bilgi yönetimi
├── arduino.py             # 🔌 Donanım arayüzü
├── qt_Arayüz/             # 🖼️ PyQt GUI arayüz dosyaları
├── modeller/              # 🧠 YOLO ağırlık modelleri
├── yardımcıDosyalar/      # 🛠️ Yardımcı/test scriptleri
├── ros2_ws/               # 🟪 ROS2 workspace
└── README.md              
```

---

## ✨ Ana Özellikler

| 🚦 Özellik                | Açıklama                                                                |
|--------------------------|------------------------------------------------------------------------ |
| Şerit & Yol Algılama     | Kamera görüntüsünden yol & şerit konumunu ve açı hesabını otomatik bulur|
| Trafik Tabelası Tanıma   | YOLO ile tabela tanıma ve sınıflandırma                                 |
| Lidar ile Engel          | Lidar ile çevresel engel & mesafe analizi                               |
| Donanım Karar Mekanizması| Arduino/Teensy kartlar ile hız, servo ve aktüatör komutları yönetimi    |
| PyQt5 GUI                | Tüm verileri/kararları görsel olarak izlemenize imkan sağlar            |

---

## 🚀 Çalıştırma Talimatları

```bash
python main.py
```

> 🧪 Alternatif olarak:  
`yardımcıDosyalar/main_logitech.py` veya kendi çalışma ortamına uygun scriptleri kullanabilirsin.

---
## 🧭 Örnek Kullanım Senaryosu

1. 🪫 Arduino, Teensy ve sensörleri bilgisayara bağla
2. 📷 Kamerayı/Lidar’ı uygun porta tak ve sistemce tanındığını doğrula
3. 🟪 ROS2 ortamını başlat (`source /opt/ros/foxy/setup.bash`)
4. Ana uygulamayı çalıştır
5. 📺 GUI üzerinden tüm verileri/kararları gözlemle

> :bulb: **Notlar:**  
> - Port, kamera/model yolu gibi donanım parametrelerini kendi setuplarına göre güncellemelisin.
> - Ek model ve bilgi için `modeller/` klasörüne göz at.
> - ROS2 node/paketlerini kendi sistemine göre düzenleyebilirsin.
