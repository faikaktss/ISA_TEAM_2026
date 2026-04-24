"""
Geliştirilmiş Şerit Algılama – BEV Tabanlı
===========================================
1. Orijinal frame → Kuş Bakışı (Perspective Transform)
2. Beyaz piksel maskesi (HLS + Canny)
3. Histogram + Kayan Pencere → Polinom Fit
4. Overlay → ters transform → orijinal perspektif
5. Kayma ve açı hesabı

Kullanım:
  python3 lane_detection.py --test /path/to/video.mp4 --output out.mp4
"""

import math, sys
import cv2
import numpy as np
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

_IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)



#Todo: Kameranın normal görünütüsünü kuş bakışı görünümüne çevirir
class PerspectiveTransformer:
    """
    KALİBRASYON (1280x720):
      alt-sol  = sol şerit alt ucu    (yakın)
      ust-sol  = sol şerit üst ucu    (uzak)
      ust-sag  = sağ şerit üst ucu   (uzak)
      alt-sag  = sağ şerit alt ucu   (yakın)
    """
    #Todo: Kamera görüntüsü üzerinde bir yamuk çizer
    # Kalibrasyon değerleri 1280×720 için yapılmıştı.
    # Yayınlanan frame 640×360 olduğundan tüm noktalar 0.5× ölçeklendi.
    # Orijinal (1280×720): [250,510], [490,295], [790,295], [1050,510]
    DEFAULT_SRC_1280x720 = np.float32([
        [250,  510],
        [490,  295],
        [790,  295],
        [1050, 510],
    ])
    # 640×360 için varsayılan (camera_node 640×360 publish ediyor)
    DEFAULT_SRC = np.float32([
        [125,  255],   # alt-sol  (250×0.5,  510×0.5)
        [245,  148],   # ust-sol  (490×0.5,  295×0.5)
        [395,  148],   # ust-sag  (790×0.5,  295×0.5)
        [525,  255],   # alt-sag  (1050×0.5, 510×0.5)
    ])

    def __init__(self, src=None, bev_w=640, bev_h=480, margin=100, frame_w=640, frame_h=360):
        self.bev_w  = bev_w #Todo: Kuş bakışı görüntüsünün genişliği
        self.bev_h  = bev_h #Todo: Kuş bakışı görüntünün yüksekliği
        if src is not None:
            self.src = np.float32(src)
        else:
            # frame boyutu 1280×720'den farklıysa src noktalarını otomatik ölçekle
            sx = frame_w / 1280.0
            sy = frame_h / 720.0
            self.src = (self.DEFAULT_SRC_1280x720 * np.float32([sx, sy])).astype(np.float32)
        #Todo: Kuş bakışı görüntünün dikdörtgen köşeleri
        self.dst    = np.float32([
            [margin,       bev_h],
            [margin,       0    ],
            [bev_w-margin, 0    ],
            [bev_w-margin, bev_h],
        ])
        self._build()

    
    def _build(self):
        self.M    = cv2.getPerspectiveTransform(self.src, self.dst)
        self.Minv = cv2.getPerspectiveTransform(self.dst, self.src)

    def warp(self, frame):
        return cv2.warpPerspective(frame, self.M, (self.bev_w, self.bev_h),
                                   flags=cv2.INTER_NEAREST)

    def unwarp(self, bev, out_w, out_h):
        return cv2.warpPerspective(bev, self.Minv, (out_w, out_h),
                                   flags=cv2.INTER_NEAREST)

    def bev_to_orig(self, pts):
        return cv2.perspectiveTransform(
            pts.reshape(-1,1,2).astype(np.float32), self.Minv
        ).reshape(-1,2).astype(np.int32)

    def draw_roi(self, frame, color=(0,165,255)):
        pts = self.src.reshape(-1,1,2).astype(np.int32)
        cv2.polylines(frame, [pts], True, color, 2)
        for p in self.src:
            cv2.circle(frame, tuple(p.astype(int)), 6, color, -1)


# Todo:  BEYAZ ŞERİT TESPİTİ

#Todo: Sadece beyaz şeritleri bulur
class WhiteLaneDetector:
    def __init__(self, brightness=165, cl=50, ch=150):
        self.bt, self.cl, self.ch = brightness, cl, ch

    def detect(self, bev):
        hls = cv2.cvtColor(bev, cv2.COLOR_BGR2HLS)
        l = hls[:, :, 1]
        _, white = cv2.threshold(l, self.bt, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(cv2.GaussianBlur(l, (5, 5), 0), self.cl, self.ch)
        return cv2.bitwise_or(white, edges)


# Todo: KAYAN PENCERE + POLİNOM
#Todo: Sağ şerit mi sol şerit mi onu algılar

class SlidingWindowFitter:
    def __init__(self, n=9, margin=80, min_pix=40, alpha=0.25):
        self.n, self.margin, self.min_pix, self.alpha = n, margin, min_pix, alpha
        self.lf_prev = self.rf_prev = None

    def _peaks(self, binary):
        h   = binary.shape[0]
        hst = np.sum(binary[h//2:,:], axis=0)#Todo: Görüntünün sadece alt yarısını al
        mid = len(hst)//2
        return int(np.argmax(hst[:mid])), int(np.argmax(hst[mid:])+mid)

    def fit(self, binary):
        h, w  = binary.shape
        wh    = h // self.n
        nz    = binary.nonzero()
        ny,nx = np.array(nz[0]), np.array(nz[1])
        lc,rc = self._peaks(binary)
        li,ri = [],[]
        dbg   = np.dstack((binary,binary,binary))*255

        for win in range(self.n):
            y0,y1 = h-(win+1)*wh, h-win*wh
            gl = ((ny>=y0)&(ny<y1)&(nx>=lc-self.margin)&(nx<lc+self.margin)).nonzero()[0]
            gr = ((ny>=y0)&(ny<y1)&(nx>=rc-self.margin)&(nx<rc+self.margin)).nonzero()[0]
            cv2.rectangle(dbg,(lc-self.margin,y0),(lc+self.margin,y1),(0,255,0),1)
            cv2.rectangle(dbg,(rc-self.margin,y0),(rc+self.margin,y1),(0,255,0),1)
            li.append(gl); ri.append(gr)
            if len(gl)>self.min_pix: lc=int(np.mean(nx[gl]))
            if len(gr)>self.min_pix: rc=int(np.mean(nx[gr]))

        li,ri = np.concatenate(li), np.concatenate(ri)
        dbg[ny[li],nx[li]]=[255,100,100]; dbg[ny[ri],nx[ri]]=[100,100,255]

        #Todo: Kıvrımlı şeritleri takip edebiliyoruz
        lf = self._poly(ny[li],nx[li],self.lf_prev)
        rf = self._poly(ny[ri],nx[ri],self.rf_prev)
        if lf is not None: self.lf_prev = lf
        elif self.lf_prev is not None: lf = self.lf_prev
        if rf is not None: self.rf_prev = rf
        elif self.rf_prev is not None: rf = self.rf_prev

        return lf, rf, dbg
        #Todo: 150 den az veri varsa şerit bulunamadı diyoruz
    def _poly(self, ys, xs, prev):
        if len(xs)<150: return None
        raw = np.polyfit(ys, xs, 2)
        return self.alpha*raw+(1-self.alpha)*prev if prev is not None else raw


# Todo: ANA ALGILAMA SINIFI

class LaneDetection:
    def __init__(self, fw=640, fh=360):
        self.fw, self.fh = fw, fh
        # frame_w/frame_h ile src noktaları otomatik ölçeklenir
        self.tf  = PerspectiveTransformer(frame_w=fw, frame_h=fh)
        self.wd  = WhiteLaneDetector()
        self.swf = SlidingWindowFitter()
        self._sk = self._sa = 0.0
        self._a  = 0.30
        self.lane_position = 0

    def process(self, frame, w, h):
        self.fw, self.fh = w, h
        bev    = self.tf.warp(frame)# Todo :Kamera görüntüsünü kuş bakışına çevir
        binary = self.wd.detect(bev)# Todo: Kuş bakışından beyaz şeritleri bulur
        lf,rf,sw_dbg = self.swf.fit(binary)

        bev_vis = self._draw_bev(bev.copy(), lf, rf)

        result = self._draw_orig(frame.copy(), lf, rf, w, h)

        self._metrics(lf, rf)

        self._hud(result, w, h)
        self.tf.draw_roi(result)

        return result, bev_vis, binary

    def _draw_bev(self, bev, lf, rf):
        if lf is None or rf is None: return bev
        h,w   = bev.shape[:2]
        py    = np.linspace(0, h-1, h)
        lx    = np.clip(np.polyval(lf,py), 0, w-1)
        rx    = np.clip(np.polyval(rf,py), 0, w-1)
        mx    = (lx+rx)/2

        fill = bev.copy()
        ptsl = np.column_stack([lx,py]).astype(np.int32)
        
        ptsr = np.column_stack([rx,py]).astype(np.int32)
        cv2.fillPoly(fill, [np.vstack([ptsl,ptsr[::-1]])], (0,180,0))
        bev = cv2.addWeighted(bev, 0.45, fill, 0.55, 0)

        for i in range(1,len(py)):
            if 0<=int(lx[i])<w: cv2.line(bev,(int(lx[i-1]),int(py[i-1])),(int(lx[i]),int(py[i])),(0,0,255),4)
            if 0<=int(rx[i])<w: cv2.line(bev,(int(rx[i-1]),int(py[i-1])),(int(rx[i]),int(py[i])),(0,255,255),4)
            if 0<=int(mx[i])<w: cv2.line(bev,(int(mx[i-1]),int(py[i-1])),(int(mx[i]),int(py[i])),(0,255,0),2)
        return bev

    def _draw_orig(self, result, lf, rf, w, h):
        if lf is None or rf is None: return result
        bh,bw = self.tf.bev_h, self.tf.bev_w
        py    = np.linspace(0, bh-1, bh)
        lx    = np.clip(np.polyval(lf,py), 0, bw-1)
        rx    = np.clip(np.polyval(rf,py), 0, bw-1)

        ov = np.zeros((bh,bw,3), dtype=np.uint8)
        ptsl = np.column_stack([lx,py]).astype(np.int32)
        ptsr = np.column_stack([rx,py]).astype(np.int32)
        cv2.fillPoly(ov, [np.vstack([ptsl,ptsr[::-1]])], (0,220,0))
        for i in range(1,len(py)):
            cv2.line(ov,(int(lx[i-1]),int(py[i-1])),(int(lx[i]),int(py[i])),(0,0,255),10)
            cv2.line(ov,(int(rx[i-1]),int(py[i-1])),(int(rx[i]),int(py[i])),(0,255,255),10)

        orig_ov = self.tf.unwarp(ov, w, h)
        result  = cv2.addWeighted(result, 1.0, orig_ov, 0.40, 0)

        lp = self.tf.bev_to_orig(ptsl)
        rp = self.tf.bev_to_orig(ptsr)
        cv2.polylines(result,[lp],False,(0,0,255),3)
        cv2.polylines(result,[rp],False,(0,255,255),3)
        return result

    def _metrics(self, lf, rf):
        bh,bw = self.tf.bev_h, self.tf.bev_w
        y = bh-1
        if lf is not None and rf is not None:
            lx = np.polyval(lf,y); rx = np.polyval(rf,y)
            raw_k = (lx+rx)/2 - bw/2
        elif lf is not None:
            raw_k = np.polyval(lf,y) - bw*0.25
        elif rf is not None:
            raw_k = np.polyval(rf,y) - bw*0.75
        else:
            raw_k = self._sk
        raw_a = math.atan(raw_k/bh)*180/math.pi if bh else 0.0
        self._sk = self._a*raw_k + (1-self._a)*self._sk
        self._sa = self._a*raw_a + (1-self._a)*self._sa
        self.lane_position = 0 if abs(self._sk)<30 else (-1 if self._sk<0 else 1)

    def _hud(self, frame, w, h):
        lbl   = {-1:"SOL",0:"MERKEZ",1:"SAG"}.get(self.lane_position,"?")
        col   = {-1:(0,80,255),0:(0,255,0),1:(0,200,255)}.get(self.lane_position,(255,255,255))
        cv2.putText(frame,f"Kayma : {self._sk:+.1f} px",(40,50), cv2.FONT_HERSHEY_SIMPLEX,0.95,(255,255,255),2)
        cv2.putText(frame,f"Aci   : {self._sa:+.1f} der",(40,90), cv2.FONT_HERSHEY_SIMPLEX,0.95,(255,255,255),2)
        cv2.putText(frame,f"Konum : {lbl}",              (40,130),cv2.FONT_HERSHEY_SIMPLEX,0.95,col,2)
        cx = w//2
        cv2.line(frame,(cx,h-60),(cx,h-10),(180,180,180),1)
        ax = max(cx-200,min(cx+200, cx+int(self._sk*w/440)))
        cv2.arrowedLine(frame,(cx,h-35),(ax,h-35),(0,200,255),3,tipLength=0.25)

    @property
    def smooth_kayma(self): return self._sk
    @property
    def smooth_aci(self):   return self._sa


# Todo: ROS2 NODE  (rclpy mevcut ise)

def _ros_node_class():
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from std_msgs.msg import Float32
        import numpy as np

        def _imgmsg_to_numpy(msg):
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1).copy()

        def _numpy_to_imgmsg(cv_image, encoding='rgb8', stamp=None, frame_id=''):
            msg = Image()
            msg.height, msg.width = cv_image.shape[:2]
            msg.encoding = encoding
            msg.is_bigendian = False
            msg.step = cv_image.shape[1] * cv_image.shape[2]
            msg.data = cv_image.tobytes()
            if stamp is not None:
                msg.header.stamp = stamp
            msg.header.frame_id = frame_id
            return msg

        class LaneDetectionNode(Node):
            def __init__(self):
                super().__init__('lane_detection_node')
                self.sub     = self.create_subscription(Image,'/zed/image_raw',self.cb,_IMAGE_QOS)
                self.p_angle = self.create_publisher(Float32,'/lane/angle',10)
                self.p_off   = self.create_publisher(Float32,'/lane/offset',10)
                self.p_debug = self.create_publisher(Image,'/lane/debug_image',10)
                self.p_bev   = self.create_publisher(Image,'/lane/bev_image',_IMAGE_QOS)
                # camera_node 640×360 publish ediyor — LaneDetection bunu bilmeli
                self.det     = LaneDetection(fw=640, fh=360)
                # Busy guard: işlem devam ederken gelen yeni frame'ler drop edilir (birikme olmaz)
                self._busy = False
                self._busy_lock = threading.Lock()
                # BEV publish pre-alloc buffer — tobytes() allocation ortadan kalkar
                _bh, _bw = self.det.tf.bev_h, self.det.tf.bev_w
                self._bev_buf = bytearray(_bh * _bw * 3)
                self._bev_np_view = np.frombuffer(self._bev_buf, dtype=np.uint8)
                self._bev_msg = Image()
                self._bev_msg.encoding = 'rgb8'
                self._bev_msg.height = _bh
                self._bev_msg.width = _bw
                self._bev_msg.step = _bw * 3
                self._bev_msg.is_bigendian = False
                self.get_logger().info('Serit algilama (BEV) basladi')
                self.get_logger().info(f'SRC (640x360): {self.det.tf.src.tolist()}')
                # Güvenlik kontrolü: src noktaları frame sınırları içinde mi?
                src = self.det.tf.src
                if src[:, 0].max() > 640 or src[:, 1].max() > 360:
                    self.get_logger().error(
                        f'[BEV HATA] src noktaları frame dışında! '
                        f'max_x={src[:,0].max():.0f} (limit 640), '
                        f'max_y={src[:,1].max():.0f} (limit 360)')

            def cb(self, msg):
                # ROS spin thread'ini bloklamadan çalış:
                # işlem sürüyorsa yeni frame'i hemen drop et (latency birikimi engellenir)
                with self._busy_lock:
                    if self._busy:
                        return
                    self._busy = True
                threading.Thread(target=self._process, args=(msg,), daemon=True).start()

            def _process(self, msg):
                try:
                    fr  = _imgmsg_to_numpy(msg)
                    bgr = cv2.cvtColor(fr, cv2.COLOR_RGB2BGR)
                    h,w = bgr.shape[:2]
                    res,bev,_ = self.det.process(bgr,w,h)

                    # BEV publish: pre-alloc buffer ile tobytes() yok
                    bev_rgb = cv2.cvtColor(bev, cv2.COLOR_BGR2RGB)
                    # FIX ADIM-F: BEV 640×480 → 320×240 (921KB → 230KB)
                    # gui_bev 2fps → 10fps+ hedefi: DDS serialize 4x azalır.
                    bev_rgb = cv2.resize(bev_rgb, (320, 240), interpolation=cv2.INTER_NEAREST)
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

                    a = Float32()
                    a.data = float(self.det.smooth_aci)
                    self.p_angle.publish(a)
                    o = Float32()
                    o.data = float(self.det.smooth_kayma)
                    self.p_off.publish(o)
                except Exception as e:
                    self.get_logger().error(str(e))
                finally:
                    with self._busy_lock:
                        self._busy = False

        return LaneDetectionNode
    except ImportError:
        return None


def main(args=None):
    import rclpy
    NodeCls = _ros_node_class()
    if NodeCls is None:
        print("[ERROR] LaneDetectionNode başlatılamadı.")
        return
    rclpy.init(args=args)
    node = NodeCls()
    try:    rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

