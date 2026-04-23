# ZED Lag + Bird's-Eye FPS/Lag Fix Planı

## Context

RealSense lag sorunu daha önce çözüldü (inline publish + LatestFrameHolder + pre-alloc buffer).
ZED kamera kodu aynı yapıya büyük ölçüde getirildi, **ancak bir kritik fark kaldı**: RS `wait_for_frames(timeout_ms=5)` ile non-blocking poll yaparken ZED hâlâ `grab()` ile **blocking** çalışıyor. Bu fark, ZED SDK'nın iç frame buffer'ında stale frame birikmesine yol açıyor ve her GC pause / Qt render gecikmesinden sonra 1–2 eski frame görüntüleniyor.

Bird's-eye lag ise iki ayrı kaynaktan besleniyor: ZED kaynak frameleri zaten biraz yaşlı geliyor, üstüne lane_detection_node içindeki `tobytes()` 921 KB allocation + warpPerspective gecikme maliyeti ekleniyor.

---

## BÖLÜM 1 — RealSense Fix Referansı

**Dosya:** [camera_node.py](ros2_ws/src/otonom_arac/otonom_arac/nodes/sensors/camera_node.py)

| # | Kritik Karar | Satır | Etki |
|---|---|---|---|
| 1 | Inline publish — timer yok, capture thread'inde doğrudan publish | :462-490 | Timer quantization delay = 0ms |
| 2 | `LatestFrameHolder` atomic swap (Lock + `_frame = None`) | :261-292 | FIFO birikimi yok, hep latest |
| 3 | `wait_for_frames(timeout_ms=5)` — non-blocking | :247 | Stale frame ≤5ms'de drain edilir |
| 4 | Pre-alloc buffer + `np.copyto()` — `tobytes()` yok | :384-393 / :486 | GC pause = 0 |
| 5 | Daemon thread + `time.sleep(0.001)` exception recovery | :361 / :498 | Hata döngüsünde CPU spike yok |

**ZED'de eksik olan:**
- `grab()` blocking → stale frame flush mekanizması yok
- ZED SDK frame timestamp'i kullanılmıyor → `capture_ns` gerçek capture zamanını göstermeyebilir
- `grab_frame()` içinde `cvtColor(RGBA→RGB)` çağrısı capture thread'inde ek geçici kopya

---

## BÖLÜM 2 — ZED Lag Bulguları

### BULGU #1 — `grab()` Blocking + Stale Frame Accumulation (KRİTİK)

**Kanıt:** [camera_node.py:201](ros2_ws/src/otonom_arac/otonom_arac/nodes/sensors/camera_node.py#L201)
```python
err = self.zed.grab(self.runtime_params)  # BLOCKING — frame gelene kadar bekler
```

**Mekanizma:**
- 30 FPS'te her `grab()` çağrısı ~33ms bloklar
- Processing pipeline'da herhangi bir gecikme olursa (GC pause, Qt render jitter) `grab()` çağrısı geç gelir
- Bu sürede ZED SDK'nın iç buffer'ı gelen frame'i saklar
- Sonraki `grab()` anında return eder ama **stale (yaşlı)** bir frame döner
- RS'nin 5ms timeout'u bu durumu otomatik handle eder: her 5ms'de bir poll → stale frame ≤5ms'de tüketilir

**Fix:** `grab()` sonrası ZED SDK frame timestamp'ini al; frame age > 40ms ise bir kez daha `grab()` yap (stale frame'i drain et):

```python
# camera_node.py → cam.grab_frame() metoduna eklenecek
def grab_frame(self, want_point_cloud=False):
    err = self.zed.grab(self.runtime_params)
    if err == sl.ERROR_CODE.SUCCESS:
        # ZED SDK'nın gerçek capture timestamp'ini al
        ts = self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)
        frame_ns = ts.get_nanoseconds()
        age_ms = (time.time_ns() - frame_ns) / 1e6
        # Stale frame: SDK buffer'da biriken eski frame → bir kez drain et
        if age_ms > 40.0:
            err2 = self.zed.grab(self.runtime_params)
            if err2 == sl.ERROR_CODE.SUCCESS:
                ts = self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)
                frame_ns = ts.get_nanoseconds()
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT, sl.MEM.CPU)
        image_numpy = self.image.get_data()
        image_rgb = cv2.cvtColor(image_numpy, cv2.COLOR_RGBA2RGB)
        return image_rgb, None, frame_ns  # capture_ns döndür
    return None, None, None
```

### BULGU #2 — `capture_ns` Gerçek Capture Zamanını Yansıtmıyor (ORTA)

**Kanıt:** [camera_node.py:416](ros2_ws/src/otonom_arac/otonom_arac/nodes/sensors/camera_node.py#L416)
```python
capture_ns = time.monotonic_ns()  # grab() + retrieve + cvtColor sonrasında
```

**Mekanizma:** `capture_ns` grab tamamlandıktan sonra set ediliyor. Gerçek capture anı daha erken; age_ms metriği olduğundan küçük görünüyor, lag izlenemez.

**Fix:** `grab_frame()` → `frame_ns` döndürsün; `_zed_capture_loop`'ta `capture_ns = frame_ns` olarak kullanılsın.

### BULGU #3 — Fazladan Geçici Kopya (DÜŞÜK)

**Kanıt:** [camera_node.py:206](ros2_ws/src/otonom_arac/otonom_arac/nodes/sensors/camera_node.py#L206) — `grab_frame()` içinde `cvtColor(RGBA→RGB)` → yeni ~760KB array, sonra capture loop `resize()` → ikinci ~691KB array.

**Fix:** `grab_frame()` sadece ham RGBA view döndürsün; cvtColor + resize capture loop'ta tek zincirde yapılsın (mevcut resize ile birleştirilebilir). *Bu minor; BULGU #1 fix sonrası önceliği düşük.*

### RealSense vs ZED Karşılaştırma Tablosu

| Kriter | RealSense ✅ | ZED ❌ | Fark |
|---|---|---|---|
| Frame acquire | `wait_for_frames(5ms)` non-blocking | `grab()` blocking | RS stale drain ≤5ms, ZED drain yok |
| Stale frame detection | Otomatik (poll hızı > frame rate) | Yok | ZED GC pause'dan sonra stale frame gösterir |
| `capture_ns` doğruluğu | `time.monotonic_ns()` sonrası (~5ms hata) | `time.monotonic_ns()` sonrası (+33ms potansiyel hata) | ZED SDK timestamp kullanılmalı |
| Inline publish | ✅ | ✅ | Aynı |
| LatestFrameHolder | ✅ | ✅ | Aynı |
| Pre-alloc buffer | ✅ | ✅ | Aynı |
| Depth disabled | ✅ | ✅ DEPTH_MODE.NONE | Aynı |
| Exception recovery | `sleep(0.001)` | `sleep(0.001)` | Aynı |

---

## BÖLÜM 3 — Bird's-Eye FPS/Lag Bulguları

### BULGU #1 — `tobytes()` Her Frame Büyük Allocation (KRİTİK)

**Kanıt:** [lane_detection_node.py:292](ros2_ws/src/otonom_arac/otonom_arac/nodes/perception/lane_detection_node.py#L292)
```python
msg.data = cv_image.tobytes()  # 640×480×3 = 921,600 bytes yeni allocation her frame
```

**Mekanizma:** BEV image (640×480) için her frame `tobytes()` → 921KB yeni nesne → GC pressure → sporadic GC pause → Qt timer jitter → BEV display lag. Camera_node.py'de bu tamamen çözüldü ama lane_detection'da hâlâ mevcut.

**Fix:** `LaneDetectionNode.__init__` içinde BEV için pre-alloc buffer; `_process()` içinde `tobytes()` yerine `np.copyto()`:

```python
# __init__'e eklenecek:
self._bev_h, self._bev_w = 480, 640
self._bev_buf = bytearray(self._bev_h * self._bev_w * 3)
self._bev_np_view = np.frombuffer(self._bev_buf, dtype=np.uint8)
self._bev_msg = Image()
self._bev_msg.encoding = 'rgb8'
self._bev_msg.height = self._bev_h
self._bev_msg.width = self._bev_w
self._bev_msg.step = self._bev_w * 3
self._bev_msg.is_bigendian = False

# _process() içinde pub(self.p_bev, bev) yerine:
bev_rgb = cv2.cvtColor(bev, cv2.COLOR_BGR2RGB)
needed = bev_rgb.nbytes
if len(self._bev_buf) != needed:
    self._bev_buf = bytearray(needed)
    self._bev_np_view = np.frombuffer(self._bev_buf, dtype=np.uint8)
    self._bev_msg.height = bev_rgb.shape[0]
    self._bev_msg.width = bev_rgb.shape[1]
    self._bev_msg.step = bev_rgb.shape[1] * bev_rgb.shape[2]
np.copyto(self._bev_np_view, bev_rgb.ravel())
self._bev_msg.data = self._bev_buf
self._bev_msg.header.stamp = msg.header.stamp
self._bev_msg.header.frame_id = msg.header.frame_id
self.p_bev.publish(self._bev_msg)
```

### BULGU #2 — `warpPerspective()` Varsayılan INTER_LINEAR (ORTA)

**Kanıt:** [lane_detection_node.py:80](ros2_ws/src/otonom_arac/otonom_arac/nodes/perception/lane_detection_node.py#L80) ve [lane_detection_node.py:82](ros2_ws/src/otonom_arac/otonom_arac/nodes/perception/lane_detection_node.py#L82)
```python
return cv2.warpPerspective(frame, self.M, (self.bev_w, self.bev_h))  # flag yok → INTER_LINEAR
return cv2.warpPerspective(bev, self.Minv, (out_w, out_h))           # flag yok → INTER_LINEAR
```

**Mekanizma:** INTER_LINEAR (default) ~8ms/frame @ 640×480. INTER_NEAREST ~4ms/frame. Lane detection ~25-35ms → warp maliyeti toplam bütçenin %25-35'i.

**Fix:** Her iki `warpPerspective` çağrısına `flags=cv2.INTER_NEAREST` ekle:
- `warp()` → kuş bakışı görsel için NEAREST yeterli kalite
- `unwarp()` → overlay çizimi için NEAREST yeterli

### BULGU #3 — Kaynak Frame Bağımlılığı: ZED Lag → BEV Lag

**Mekanizma:**
```
ZED grab stale (0-40ms lag)
  → ZED publish
    → lane_detection frame receive
      → processing 25-35ms
        → BEV publish (tobytes 2-4ms)
          → GUI bev_callback
            → Qt timer quantization (0-33ms)
              → display
TOPLAM: 30-115ms (ZED stale + lane proc + Qt)
```

**Soru: ZED fix → BEV otomatik düzelir mi?**

**Kısmen. Tam düzelmez.** ZED fix, kaynak frame age'ini 0-40ms → 0-5ms seviyesine çeker (~35ms kazanım). Ancak lane detection'ın kendi 25-35ms işlem süresi + Qt timer 0-33ms quantization'ı yapısal ve ZED'den bağımsız. BEV için BULGU #1 ve #2 fix'leri zorunlu.

---

## BÖLÜM 4 — Uygulama Sırası

```
Adım 1: ZED stale frame drain — camera_node.py
  grab_frame() → timestamp al, age > 40ms → extra grab
  _zed_capture_loop → capture_ns = gerçek ZED timestamp
  Beklenen kazanım: ZED render age_ms < 50ms (şu an 50-100ms spike'ları var)

Adım 2: BEV tobytes() → pre-alloc buffer — lane_detection_node.py
  LaneDetectionNode: _bev_buf, _bev_np_view, _bev_msg init'te
  _process(): tobytes() kaldır, np.copyto() kullan
  Beklenen kazanım: GC pause ortadan kalkar, BEV lag daha stabil

Adım 3: warpPerspective INTER_NEAREST — lane_detection_node.py
  PerspectiveTransformer.warp() + unwarp() → flags=INTER_NEAREST
  Beklenen kazanım: ~4ms/frame tasarruf → lane detection 25-35ms → ~21-31ms

Adım 4: Ölçüm + doğrulama
  ros2 topic echo /perf/summary → zed_interval_max, zed_stutter_cnt, bev render age
```

---

## Kritik Dosyalar

| Dosya | Değişiklik | Adım |
|---|---|---|
| [camera_node.py:200-211](ros2_ws/src/otonom_arac/otonom_arac/nodes/sensors/camera_node.py#L200) | `grab_frame()` → timestamp + stale drain | 1 |
| [camera_node.py:414-418](ros2_ws/src/otonom_arac/otonom_arac/nodes/sensors/camera_node.py#L414) | `_zed_capture_loop` → `capture_ns = frame_ns` | 1 |
| [lane_detection_node.py:298-356](ros2_ws/src/otonom_arac/otonom_arac/nodes/perception/lane_detection_node.py#L298) | Pre-alloc BEV buffer init + publish | 2 |
| [lane_detection_node.py:80](ros2_ws/src/otonom_arac/otonom_arac/nodes/perception/lane_detection_node.py#L80) | `warp()` → `INTER_NEAREST` | 3 |
| [lane_detection_node.py:82](ros2_ws/src/otonom_arac/otonom_arac/nodes/perception/lane_detection_node.py#L82) | `unwarp()` → `INTER_NEAREST` | 3 |

---

## BÖLÜM 5 — Kabul Kriterleri

```
/perf/summary ile doğrulama:

ZED:
  zed_interval_max < 40ms   (stutter yok)
  zed_stutter_cnt = 0       (10 dk koşu sonrası)
  gui_zed_age < 80ms        (render age sabit, spike yok)

BEV:
  bev render age < 120ms    (lane proc 25ms + transport + Qt timer)
  bev render age kararlı    (spike < ±20ms)
  zed_drop = 0              (kaynak frame kaybı yok)

Regresyon kontrolü:
  rs_cap ~= 30fps            (RealSense bozulmadı)
  rs_stutter_cnt = 0         (RS regresyon yok)
  zed_cap ~= 30fps           (ZED FPS korundu)
```
