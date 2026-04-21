# GUI FPS Düşüşü — Kök Neden Analizi & Adım Adım Çözüm Planı

> **Durum:** Her adım uygulanmadan önce onay istenir.  
> **Donanım sorunu değil** — tüm kök nedenler yazılımsal.

---

## Kök Neden (Tek Cümle)

`camera_node._publish_callback`, GUI açılınca DDS'in 3 aktif subscriber'a ~7.6 MB/tick IPC transfer başlatmasıyla 33 ms timer bütçesini aşıyor; uzayan callback Python GIL'i tutuyor ve ZED/RS capture daemon thread'lerinin `cvtColor` adımlarını geciktiriyor → hardware frame'leri SDK buffer'da birikiyor → Python tarafında FPS düşüşü görülüyor.

---

## Uçtan Uca Data Flow

```
[ZED grab() daemon-thread]     [RS wait_for_frames() daemon-thread]
    ↓ LatestFrameHolder                ↓ LatestFrameHolder
    ↓                                  ↓
  _publish_callback (30 Hz ROS2 timer — TEK callback, DARBOĞAZ)
    ├─ resize 1920×1080 → 640×360     ~2 ms GIL
    ├─ tobytes ZED raw 1920×1080      ~8 ms GIL + IPC
    ├─ tobytes ZED preview 640×360    ~1 ms GIL + IPC
    ├─ [Her 3. frame] tolist() 97K    10-40 ms GIL  ← spike
    └─ tobytes RS 640×480             ~2 ms GIL + IPC

  GUI kapalı → subscriber yok → publish() hızlı döner
  GUI açılınca → 3 aktif sub → serialize+IPC her frame
  Toplam: ~7.6 MB/tick × 30 Hz ≈ 230 MB/s IPC
  Callback süresi 33 ms bütçeyi aşar → GIL uzun tutulur → capture thread'ler bekler
```

---

## Bulgular

| # | Dosya:Satır | Sorun | GUI yokken | GUI açılınca |
|---|------------|-------|-----------|-------------|
| B1 | `camera_node.py:334-380` | Tek callback'te 3 frame serialize + point cloud tolist() | Hızlı (subscriber yok) | IPC devreye girer, >33ms |
| B2 | `camera_node.py:165-168` | ZED HD1080 (1920×1080) — lane için aşırı çözünürlük | 6 MB/frame | 6 MB/frame × 3 sub |
| B3 | `camera_node.py:355-367` | `tolist()` 97K elem → 10-40 ms GIL spike | Her 3 frame | Her 3 frame, timer aşılır |
| B4 | `gui_node.py:197` | zed_callback'te gereksiz `cv2.resize(640×360→640×360)` | Yok | ROS spin thread'de her frame |
| B5 | `lane_detection_node.py:301` | debug_image publish (ekstra IPC) | Yok (sub yok) | GUI bev_image sub → lane callback artar |
| B6 | `object_detection_node.py:159` | YOLO imgsz=736, CPU baskısı | Var | Artar (CPU rekabeti) |

---

## Adım Adım Çözüm Planı

### ADIM 1 — Ölçüm altyapısı kur (perf/metrics.py)
**Amaç:** Önce ölç, sonra düzelt. Baseline al.  
**Dosya:** `ros2_ws/src/otonom_arac/otonom_arac/perf/metrics.py` (yeni)  
**Ne yapılır:** `FPSMeter`, `LatencyMeter`, `SummaryLogger` sınıfları oluşturulur.  
`camera_node._publish_callback`, `_zed_capture_loop`, `_rs_capture_loop` içine entegre edilir.  
Her 1 sn'de bir log: `zed_capture=29.8fps | rs_capture=29.5fps | zed_publish=28.1fps`

---

### ADIM 2 — ZED çözünürlüğünü HD1080 → HD720'ye düşür (B2)
**Dosya:** `sensors/camera_node.py:167`  
**Değişiklik:**
```python
# ÖNCE:
init_params.camera_resolution = sl.RESOLUTION.HD1080
# SONRA:
init_params.camera_resolution = sl.RESOLUTION.HD720   # 1280×720
```
**Etki:** Serialization 6 MB/frame → 2.8 MB/frame (%53 azalma). Lane detection için yeterli.

---

### ADIM 3 — Point cloud throttle artır (B3)
**Dosya:** `sensors/camera_node.py:355`  
**Değişiklik:**
```python
# ÖNCE:
if point_cloud is not None and self._pc_counter % 3 == 0:
# SONRA:
if point_cloud is not None and self._pc_counter % 9 == 0:   # ~3.3 Hz
```
**Etki:** tolist() GIL spike sıklığı 3x azalır.

---

### ADIM 4 — GUI zed_callback'ten gereksiz resize kaldır (B4)
**Dosya:** `gui/gui_node.py:197` ve `imgmsg_to_cv2` (satır 292-297)  
**Değişiklik:** `cv2.resize(frame, (640, 360))` satırını sil; `.copy()` kaldır.  
**Neden güvenli:** Camera node zaten 640×360 publish ediyor. Lock koruyor.

---

### ADIM 5 — Lane detection debug_image publish kaldır (B5)
**Dosya:** `perception/lane_detection_node.py:301`  
**Değişiklik:**
```python
# pub(self.p_debug, res)   ← kaldır (production'da gereksiz)
pub(self.p_bev, bev)
```
**Etki:** Lane node IPC trafik azalır, callback süresi kısalır.

---

### ADIM 6 — YOLO imgsz küçült (B6)
**Dosya:** `perception/object_detection_node.py:159`  
**Değişiklik:**
```python
# ÖNCE:
results = self.model.predict(..., imgsz=(736, 736), ...)
# SONRA:
results = self.model.predict(..., imgsz=640, ...)
```
**Etki:** ~%20-25 inference hızı artışı, CPU baskısı azalır.

---

### ADIM 7 — camera_node._publish_callback'i ZED/RS ayrı timer'lara böl (B1 kalıcı fix)
**Dosya:** `sensors/camera_node.py`  
**Değişiklik:** Tek `_publish_callback` kaldırılır; üç ayrı timer oluşturulur:
- `_zed_timer` (30 Hz) → sadece ZED raw + preview
- `_rs_timer` (30 Hz) → sadece RS
- `_pc_timer` (5 Hz) → sadece point cloud

**Etki:** Her timer callback bütçesi düşer, GIL spike'ları izole edilir.

---

## Başarı Kriterleri

| Metrik | Hedef |
|--------|-------|
| `zed_capture_fps` GUI açık/kapalı farkı | <%10 |
| `rs_capture_fps` GUI açık/kapalı farkı | <%10 |
| `_publish_callback` süresi | <25 ms (33 ms bütçe altında) |
| GUI render FPS | Sabit ~30 FPS |
| 10 dk koşu | UI kilitlenmiyor, memory leak yok |

**Test:**
```bash
ros2 topic hz /zed/preview /realsense/image_raw /lane/bev_image
# GUI kapalı vs açık karşılaştır
```

---

## Sıradaki Adım

**ADIM 1'den başlanacak** — onay bekleniyor.





Camera_node.py da kaç fps oluyor izlemek için bunu çalıştır (faik)
ros2 topic echo /perf/summary