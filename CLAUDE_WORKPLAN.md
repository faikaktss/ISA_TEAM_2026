# ISA_TEAM_2025-2026_ROS2_HUMBLE — Claude Çalışma Planı

## Amaç
Her `.py` dosyasını **ISA_TEAM_2025-2026 algoritma mantığına göre** adım adım incelemek, eksikleri tespit etmek, düzeltmek.  
Minimum token ile **dosya bazlı** ilerlemek.

---

## Repo Yapısı
```
ros2_ws/src/otonom_arac/otonom_arac/nodes/
  control/
    control_node.py
    teensy_node.py
  gui/
    gui_node.py        ← KRİTİK: arayüz hatası
  perception/
    lane_detection_node.py
    object_detection_node.py
  sensors/
    camera_node.py
    encoder_node.py
    joystick_node.py
    lidar_node.py
  testing/
    video_player_node.py
```
Referans: `ISA_TEAM_2024-2025` (eski repo) algoritma karşılaştırması için.

---

## Sistem Akışı
1. **Arduino** → 4 sensör verisi okur  
2. **ROS2** → bu 4 veriyi işler  
3. **Teensy** → 5 veri paketi gönderir (4 + 1 türetilmiş)

### Temel Sorunlar
- `gui_node.py`: UI akışı / subscriber-publisher / thread çakışması
- Arduino→Teensy: paket format / mapping / ölçekleme tutarsızlıkları
- Node'lar arası topic isimleri, message tipi, QoS ve veri tipleri tutarsız

---

## Arduino (4 veri) → Teensy (5 veri) Dönüşüm Spec
### Arduino ham veriler
- `A1:` ? — type: ?
- `A2:` ? — type: ?
- `A3:` ? — type: ?
- `A4:` ? — type: ?

### Teensy komut paketi
- `T1:` ? — type: ?
- `T2:` ? — type: ?
- `T3:` ? — type: ?
- `T4:` ? — type: ?
- `T5:` ? *(türetilmiş)* — type: ?

**Doldurulacak:**
- Baudrate + port isimleri
- Paket formatı: CSV / binary / delimiter / CRC?
- Her verinin aralığı + birimi

---

## GUI Hedef Davranış
- Subscribe ettiği topic'ler: ?
- Publish ettiği topic'ler: ?
- UI thread/timer yönetimi: ?
- ROS2 spin + UI event loop çakışması var mı?

**Doldurulacak:**
- Framework: PyQt / Tkinter / rqt / custom?
- Hata mesajı / stacktrace
- Publish/subscribe topic + message tipleri

---

## Claude Yanıt Formatı (zorunlu)
```
FILESCOPE: <dosya yolu>
ROLE:      <1 cümle>
IO:        <publish/subscribe/serial tablosu>
ISSUES:    <max 5 madde, kritikler önce>
FIX:       <max 5 madde, uygulanabilir>
PATCH:     <sadece gerekli kod parçaları>
```
Dosyayı görmeden konuşma → `NEED:` ile gereken bilgiyi iste.

---

## Tarama Sırası
1. `nodes/gui/gui_node.py`            ← KRİTİK
2. `nodes/control/teensy_node.py`
3. `nodes/control/control_node.py`
4. `nodes/sensors/joystick_node.py`
5. `nodes/sensors/encoder_node.py`
6. `nodes/sensors/lidar_node.py`
7. `nodes/sensors/camera_node.py`
8. `nodes/perception/lane_detection_node.py`
9. `nodes/perception/object_detection_node.py`
10. `nodes/testing/video_player_node.py`

---

## Çalışma Kuralları
- Tahmin yapma — dosya içeriğine göre konuş
- Uzun açıklama yazma — sadece uygulanabilir maddeler
- Bir seferde **1 dosya** incele