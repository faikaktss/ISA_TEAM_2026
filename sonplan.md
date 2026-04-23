# FPS Final Audit — Tam Repo Tarama Planı

## Context

Kullanıcı "GUI açılınca FPS düşüşü geri gelmesin" seviyesinde güvence istiyor.
Önceki oturumda bazı race condition'lar fix edildi (ad9b422), ancak audit sırasında
hâlâ açık olan bug'lar + performans sorunları bulundu. Bu plan tüm repo üzerinde
(sensors/, perception/, control/, gui/, perf/, testing/, archived/) yapılan final
taramanın bulguları ve uygulanacak fix'leri sıralar.

Başlangıç durumu: commit `ad9b422` — object_detection `_busy_lock` fix, pc_lock eklendi.

---

## Repo Envanteri (Kapsam Onayı)

Taranan tüm dosyalar:
- `nodes/gui/gui_node.py`
- `nodes/sensors/camera_node.py`
- `nodes/sensors/encoder_node.py`
- `nodes/sensors/lidar_node.py`
- `nodes/sensors/joystick_node.py`
- `nodes/control/control_node.py`
- `nodes/control/teensy_node.py`
- `nodes/perception/lane_detection_node.py`
- `nodes/perception/object_detection_node.py`
- `perf/metrics.py`
- `nodes/testing/__init__.py` (video_player_node.py yalnızca build/ altında, src'de yok)
- `archived/qt_Arayüz/` (passif, çalışmıyor — risk yok)
- `launch/otonom_arac_launch.py`
- `Dockerfile`, `docker-compose.yml`

**"False alarm" olmayanlar:** archived/, test_flake8.py, test_pep257.py —
bunlar runtime'da çalışmaz, FPS riski sıfır.

---

## BÖLÜM 1 — Onaylı Bug'lar (Kod Kanıtlı, Mutlaka Fix)

### BUG-1 ★★★ CRITICAL — DBSCAN Worker Race Condition
**Dosya:** `gui_node.py:260-266`

```python
# MEVCUT (YANLIŞ):
if not self._dbscan_busy:
    with self._dbscan_spawn_lock:
        if not self._dbscan_busy:          # _busy henüz True değil!
            threading.Thread(...).start()  # thread başlatıldı

def _dbscan_worker(self):
    self._dbscan_busy = True  # ← GEÇ! Thread zaten çalışıyor
```

**Sorun:** `_dbscan_busy = True` worker içinde set ediliyor; thread başlatılıp
bu satıra ulaşana kadar yeni bir lidar callback `_busy == False` görüp ikinci
worker spawn edebilir. İki worker aynı `self._dbscan` instance'ına `fit()` çağırırsa
sklearn instance state bozulur → tanımsız davranış + intermittent GUI donması.

**Fix:**
```python
# DÜZELTME — spawn_lock içinde _busy=True, THREAD BAŞLAMADAN:
with self._dbscan_spawn_lock:
    if not self._dbscan_busy:
        self._dbscan_busy = True          # ← lock içinde, thread öncesi
        threading.Thread(target=self._dbscan_worker, daemon=True).start()

def _dbscan_worker(self):
    # self._dbscan_busy = True  ← bu satırı SİL
    try:
        dbscan = DBSCAN(eps=200, min_samples=4)  # local instance, thread-safe
        labels = dbscan.fit(points_np).labels_
        ...
    finally:
        self._dbscan_busy = False
```

Dış `if not self._dbscan_busy:` kontrolünü de kaldır (lock içindeki yeterli).

**Etki:** Lidar 10 Hz → saat başına ~36000 spawn attempt. Spikes kaybolur.

---

### BUG-2 ★★ HIGH — _last_point_cloud Lock Eksik
**Dosya:** `camera_node.py:399-402`

```python
# MEVCUT (YANLIŞ):
def _publish_pc_callback(self):          # ROS executor thread
    pc = self._last_point_cloud          # ← oku
    if pc is None: return
    self._last_point_cloud = None        # ← sil  (ayrı iki op, lock yok)
```

ZED capture thread (ayrı thread) şu anda `_last_point_cloud` yazıyor.
Bu iki thread herhangi bir lock olmadan erişiyor → read-after-partial-write.

**Fix:**
```python
# camera_node.__init__:
self._pc_data_lock = threading.Lock()

# _zed_capture_loop:
with self._pc_data_lock:
    self._last_point_cloud = point_cloud

# _publish_pc_callback:
with self._pc_data_lock:
    pc = self._last_point_cloud
    self._last_point_cloud = None
```

---

### BUG-3 ★ MEDIUM — control_node Serial Timeout = 1 s
**Dosya:** `control_node.py:72`

```python
self.arduino = serial.Serial(arduino_port, arduino_baudrate, timeout=1)
# imu_timer: 50 Hz (20 ms bütçe)
```

`imu_okuma()` `in_waiting > 0` kontrolü yapıyor, bu yüzden *normalde* bloklama yok.
Ancak: kablo koparsa / buffer'da yarım satır gelirse `readline()` tam 1 saniye bloklar.
Bu süre boyunca ROS executor'ın tüm callback'leri (lane, detection, control_loop) durur.

**Fix:**
```python
self.arduino = serial.Serial(arduino_port, arduino_baudrate, timeout=0.01)
```

---

### BUG-4 ★ MEDIUM — camera_node destroy_node() Thread Join Eksik
**Dosya:** `camera_node.py:417-423`

```python
def destroy_node(self):
    self._running = False
    # ← Thread join yok! daemon thread'ler OS tarafından kill edilir
    super().destroy_node()
```

Uygulama kapanırken ZED/RS thread'leri `grab()` ortasında kesilirse SDK state
bozuk kalır; sonraki başlatmada kamera "already open" hatası verebilir.

**Fix:**
```python
def destroy_node(self):
    self._running = False
    for t in (self._zed_thread, self._rs_thread):
        if t and t.is_alive():
            t.join(timeout=1.0)
    self._zed_timer.cancel()
    self._rs_timer.cancel()
    self._pc_timer.cancel()
    super().destroy_node()
```

---

## BÖLÜM 2 — Performans İyileştirmeleri (Bug Değil, Ama Faydalı)

### PERF-1 — Lane Detection Dual cvtColor
**Dosya:** `lane_detection_node.py:89,91`

```python
l    = cv2.cvtColor(bev, cv2.COLOR_BGR2HLS)[:,:,1]  # op-1
gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)         # op-2 (ayrı)
```

Her iki dönüşüm de tüm frame üstünde yapılıyor. Küçük optimizasyon:
BGR→HLS zaten L kanalını verir; gray için HLS'den L kanalı yeterli çoğu durumda.

**Fix (minimal):**
```python
hls  = cv2.cvtColor(bev, cv2.COLOR_BGR2HLS)
l    = hls[:, :, 1]                                  # HLS'den L kanalı
gray = l                                             # Canny için L yeterli
```

Tek `cvtColor` çağrısına iner. ~2-3 ms/frame tasarruf.

---

### PERF-2 — GUI Spin Thread daemon=True (Shutdown Risk)
**Dosya:** `gui_node.py:510`

```python
spin_thread = threading.Thread(
    target=lambda: rclpy.spin(ros_node),
    daemon=True                                      # ← GUI kapanınca anında kill
)
```

ROS2 spin thread'in aniden kesilmesi bazı cleanup callback'lerinin çalışmamasına yol açar.
Ciddi performans etkisi yok ama shutdown'da resource leak mümkün.

**Fix:**
```python
spin_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=False)
spin_thread.start()
# Qt app kapanmadan önce:
rclpy.shutdown()
spin_thread.join(timeout=2.0)
```

---

### PERF-3 — GUI _show_frame İçinde scaled() Her Frame
**Dosya:** `gui_node.py:~495`

Her frame için `pixmap.scaled(...)` yeniden boyutlandırma yapılıyor.
Frame boyutu label boyutuna eşit veya küçükse gereksiz operasyon.

**Fix:** Frame'i subscriber callback'te `label.size()`'a göre resize et,
`_show_frame`'de sadece `setPixmap(QPixmap.fromImage(...))` yap.
Etkisi düşük (~2 ms/frame) ama birikiyor.

---

## BÖLÜM 3 — PASS / RISK / FAIL Checklist

| Kriter | Durum | Kanıt |
|--------|-------|-------|
| UI thread bloklanmıyor | ✅ PASS | Qt timer → update_ui, ROS spin ayrı thread |
| Capture/Process/Render ayrımı | ✅ PASS | LatestFrameHolder decoupling |
| Latest-frame buffer | ✅ PASS | LatestFrameHolder.get() consume |
| Render throttle | ✅ PASS | QTimer 33ms (~30 FPS) |
| I/O izolasyonu | ✅ PASS | Logging throttled, disk I/O yok |
| DBSCAN thread güvenliği | ❌ FAIL | BUG-1 — spawn race |
| Point cloud thread güvenliği | ❌ FAIL | BUG-2 — lock eksik |
| Serial timeout | ⚠️ RISK | BUG-3 — 1s blok riski |
| Shutdown cleanup | ⚠️ RISK | BUG-4 — join eksik |
| Exception/retry backoff | ✅ PASS | time.sleep(0.001) hata durumunda |
| İki kamera bağımsızlığı | ✅ PASS | Ayrı thread + holder |
| YOLO race condition | ✅ PASS | ad9b422'de fix edildi |
| PC callback lock | ✅ PASS | object_detection'da eklendi |

---

## BÖLÜM 4 — Regresyon Senaryoları

| # | Senaryo | Risk | Korunma |
|---|---------|------|---------|
| R1 | Debug log açılır | Logging throttle var, risk düşük | Log seviyesi warn/error'da bırak |
| R2 | YOLO inference açılır | _busy flag korur, no queue buildup | OK |
| R3 | Çözünürlük artar (HD1080) | tolist() 5 Hz'de; 200ms bütçe yeterli | HD720'de kal |
| R4 | Depth/point cloud açılır | 5 Hz timer izole; BUG-2 fix sonrası OK | BUG-2'yi fix et |
| R5 | GUI pencere resize | repaint → label.scaled() her frame | PERF-3 fix'i |
| R6 | Kamera kısa süre frame vermez | capture_loop: sleep(0.001) on exception | OK |
| R7 | İki lidar callback aynı anda | BUG-1 olmadan ikinci worker spawn | BUG-1 fix et |
| R8 | Arduino kablosu çekilir | timeout=1s → 1s blok; BUG-3 fix gerekli | BUG-3'ü fix et |
| R9 | CPU ani %100 | GIL: tolist() 5 Hz; diğerleri CPU-bound değil | Kabul edilebilir |
| R10 | Uygulama kapatılır | join() eksik → ZED SDK state bozuk | BUG-4'ü fix et |
| R11 | DBSCAN sırasında yeni lidar mesajı | BUG-1 olmadan çift worker | BUG-1 fix et |

---

## BÖLÜM 5 — Fix Uygulama Sırası

| Öncelik | Bug/Perf | Dosya | Satır | Süre |
|---------|----------|-------|-------|------|
| 1 | BUG-1 DBSCAN spawn race | `gui/gui_node.py` | 260-266, 265 | 20 dk |
| 2 | BUG-2 pc_data_lock | `sensors/camera_node.py` | ~370, 399-402 | 15 dk |
| 3 | BUG-3 serial timeout | `control/control_node.py` | 72 | 2 dk |
| 4 | BUG-4 thread join | `sensors/camera_node.py` | 417-423 | 10 dk |
| 5 | PERF-1 dual cvtColor | `perception/lane_detection_node.py` | 89-91 | 5 dk |
| 6 | PERF-2 spin daemon | `gui/gui_node.py` | 510 | 10 dk |
| 7 | PERF-3 scaled() | `gui/gui_node.py` | ~495 | 15 dk |

---

## BÖLÜM 6 — Doğrulama

```bash
# 1. Build + Launch
colcon build && source install/setup.bash
ros2 launch otonom_arac otonom_arac_launch.py

# 2. FPS monitor (ayrı terminal)
ros2 topic hz /zed/preview /lane/bev_image /realsense/image_raw

# 3. Perf summary
ros2 topic echo /perf/summary

# 4. GUI kapatma testi (BUG-4)
# GUI'yi kapat → "ZED already open" hatası olmadan yeniden başlatılabilmeli

# 5. Arduino kablosu çekme testi (BUG-3)
# Kablo çekilince control_loop max 10ms'de dönmeli (1s değil)

# 6. DBSCAN spike testi (BUG-1)
# Lidar bağlıyken 2 dakika izle → GUI'de donma/spike olmamalı
# ros2 topic hz /lidar/scan → 10 Hz stabil olmalı

# 7. Hedef metrikler
# /zed/preview: 29-30 FPS
# /lane/bev_image: 28-30 FPS
# /perf/summary: zed_cb violations=0
# GUI: 30 FPS, donma yok
```

---

## Son Karar

**Bu repo şu an FPS sorunu açısından güvenli mi?** → **HAYIR** (4 onaylı bug açık)

Önce bunlar:
1. **BUG-1** — DBSCAN spawn race (GUI donması riski, %100 tekrarlanabilir yüksek lidar frekansta)
2. **BUG-2** — camera_node pc_data_lock eksik (race condition, veri tutarsızlığı)
3. **BUG-3** — serial timeout=1s (Arduino kesilince 1s blok, tüm callback zinciri durur)

BUG-4 ve PERF'ler bu üç fix sonrasında uygulanabilir.
