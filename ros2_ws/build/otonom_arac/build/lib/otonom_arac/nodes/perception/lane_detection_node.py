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

import math, sys, argparse
import cv2
import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# 1. PERSPECTIVE TRANSFORMER
# ─────────────────────────────────────────────────────────────────────────────

class PerspectiveTransformer:
    """
    KALİBRASYON (1280x720):
      alt-sol  = sol şerit alt ucu    (yakın)
      ust-sol  = sol şerit üst ucu    (uzak)
      ust-sag  = sağ şerit üst ucu   (uzak)
      alt-sag  = sağ şerit alt ucu   (yakın)
    """
    DEFAULT_SRC = np.float32([
        [250,  510],   # alt-sol
        [490,  295],   # ust-sol
        [790,  295],   # ust-sag
        [1050, 510],   # alt-sag
    ])

    def __init__(self, src=None, bev_w=640, bev_h=480, margin=100):
        self.bev_w  = bev_w
        self.bev_h  = bev_h
        self.src    = np.float32(src) if src is not None else self.DEFAULT_SRC.copy()
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
        return cv2.warpPerspective(frame, self.M, (self.bev_w, self.bev_h))

    def unwarp(self, bev, out_w, out_h):
        return cv2.warpPerspective(bev, self.Minv, (out_w, out_h))

    def bev_to_orig(self, pts):
        return cv2.perspectiveTransform(
            pts.reshape(-1,1,2).astype(np.float32), self.Minv
        ).reshape(-1,2).astype(np.int32)

    def draw_roi(self, frame, color=(0,165,255)):
        pts = self.src.reshape(-1,1,2).astype(np.int32)
        cv2.polylines(frame, [pts], True, color, 2)
        for p in self.src:
            cv2.circle(frame, tuple(p.astype(int)), 6, color, -1)


# ─────────────────────────────────────────────────────────────────────────────
# 2. BEYAZ ŞERİT TESPİTİ
# ─────────────────────────────────────────────────────────────────────────────

class WhiteLaneDetector:
    def __init__(self, brightness=165, cl=50, ch=150):
        self.bt, self.cl, self.ch = brightness, cl, ch

    def detect(self, bev):
        l   = cv2.cvtColor(bev, cv2.COLOR_BGR2HLS)[:,:,1]
        _,white = cv2.threshold(l, self.bt, 255, cv2.THRESH_BINARY)
        gray  = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv2.GaussianBlur(gray,(5,5),0), self.cl, self.ch)
        return cv2.bitwise_or(white, edges)


# ─────────────────────────────────────────────────────────────────────────────
# 3. KAYAN PENCERE + POLİNOM
# ─────────────────────────────────────────────────────────────────────────────

class SlidingWindowFitter:
    def __init__(self, n=9, margin=80, min_pix=40, alpha=0.25):
        self.n, self.margin, self.min_pix, self.alpha = n, margin, min_pix, alpha
        self.lf_prev = self.rf_prev = None

    def _peaks(self, binary):
        h   = binary.shape[0]
        hst = np.sum(binary[h//2:,:], axis=0)
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

        lf = self._poly(ny[li],nx[li],self.lf_prev)
        rf = self._poly(ny[ri],nx[ri],self.rf_prev)
        if lf is not None: self.lf_prev = lf
        elif self.lf_prev is not None: lf = self.lf_prev
        if rf is not None: self.rf_prev = rf
        elif self.rf_prev is not None: rf = self.rf_prev

        return lf, rf, dbg

    def _poly(self, ys, xs, prev):
        if len(xs)<150: return None
        raw = np.polyfit(ys, xs, 2)
        return self.alpha*raw+(1-self.alpha)*prev if prev is not None else raw


# ─────────────────────────────────────────────────────────────────────────────
# 4. ANA ALGILAMA SINIFI
# ─────────────────────────────────────────────────────────────────────────────

class LaneDetection:
    def __init__(self, fw=1280, fh=720):
        self.fw, self.fh = fw, fh
        self.tf  = PerspectiveTransformer()
        self.wd  = WhiteLaneDetector()
        self.swf = SlidingWindowFitter()
        self._sk = self._sa = 0.0
        self._a  = 0.30
        self.lane_position = 0

    def process(self, frame, w, h):
        self.fw, self.fh = w, h
        bev    = self.tf.warp(frame)
        binary = self.wd.detect(bev)
        lf,rf,sw_dbg = self.swf.fit(binary)

        # ── BEV görsel ───────────────────────────────────────────────────
        bev_vis = self._draw_bev(bev.copy(), lf, rf)

        # ── Overlay → orijinal perspektif ────────────────────────────────
        result = self._draw_orig(frame.copy(), lf, rf, w, h)

        # ── Metrik ───────────────────────────────────────────────────────
        self._metrics(lf, rf)

        # ── HUD ──────────────────────────────────────────────────────────
        self._hud(result, w, h)
        self.tf.draw_roi(result)

        return result, bev_vis, binary

    # ── BEV üzerine şerit çiz ────────────────────────────────────────────
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

    # ── Orijinal perspektif overlay ──────────────────────────────────────
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

    # ── Metrik hesabı ────────────────────────────────────────────────────
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

    # ── HUD ─────────────────────────────────────────────────────────────
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


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 NODE  (rclpy mevcut ise)
# ─────────────────────────────────────────────────────────────────────────────

def _ros_node_class():
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from std_msgs.msg import Float32
        from cv_bridge import CvBridge

        class LaneDetectionNode(Node):
            def __init__(self):
                super().__init__('lane_detection_node')
                self.sub     = self.create_subscription(Image,'/camera/image_raw',self.cb,10)
                self.p_angle = self.create_publisher(Float32,'/lane/angle',10)
                self.p_off   = self.create_publisher(Float32,'/lane/offset',10)
                self.p_debug = self.create_publisher(Image,'/lane/debug_image',10)
                self.p_bev   = self.create_publisher(Image,'/lane/bev_image',10)
                self.bridge  = CvBridge()
                self.det     = LaneDetection()
                self.get_logger().info('Serit algilama (BEV) basladi')
                self.get_logger().info(f'SRC: {self.det.tf.src.tolist()}')

            def cb(self, msg):
                try:
                    fr  = self.bridge.imgmsg_to_cv2(msg,'rgb8')
                    bgr = cv2.cvtColor(fr, cv2.COLOR_RGB2BGR)
                    h,w = bgr.shape[:2]
                    res,bev,_ = self.det.process(bgr,w,h)

                    def pub(p,img):
                        p.publish(self.bridge.cv2_to_imgmsg(
                            cv2.cvtColor(img,cv2.COLOR_BGR2RGB),'rgb8'))

                    pub(self.p_debug, res)
                    pub(self.p_bev,   cv2.resize(bev,(w,h)))

                    a=Float32(); a.data=float(self.det.smooth_aci);   self.p_angle.publish(a)
                    o=Float32(); o.data=float(self.det.smooth_kayma); self.p_off.publish(o)
                except Exception as e:
                    self.get_logger().error(str(e))

        return LaneDetectionNode
    except ImportError:
        return None


def main(args=None):
    import rclpy
    NodeCls = _ros_node_class()
    rclpy.init(args=args)
    node = NodeCls()
    try:    rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ─────────────────────────────────────────────────────────────────────────────
# VIDEO TEST MODU
# ─────────────────────────────────────────────────────────────────────────────

def main_test(video_path, output_path=None, show=False):
    cap = cv2.VideoCapture(video_path)
    W   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    FPS = cap.get(cv2.CAP_PROP_FPS)
    BW  = 640
    print(f"Giris: {W}x{H} @{FPS:.1f}fps")

    writer = None
    if output_path:
        writer = cv2.VideoWriter(output_path,
                                 cv2.VideoWriter_fourcc(*'mp4v'),
                                 FPS, (W+BW, H))
    det = LaneDetection(W, H)
    n   = 0
    while True:
        ret, frame = cap.read()
        if not ret: break
        n += 1
        result, bev_vis, _ = det.process(frame, W, H)

        panel = cv2.resize(bev_vis, (BW, H))
        cv2.putText(panel,"KUS BAKISI",(20,40),cv2.FONT_HERSHEY_SIMPLEX,1.2,(255,255,0),2)
        cv2.putText(panel,f"Kayma:{det.smooth_kayma:+.1f}",(20,H-45),cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,255,200),2)
        cv2.putText(panel,f"Aci:{det.smooth_aci:+.1f}",(20,H-15),cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,255,200),2)

        canvas = np.zeros((H, W+BW, 3), dtype=np.uint8)
        canvas[:,:W]  = result
        canvas[:,W:]  = panel
        cv2.line(canvas,(W,0),(W,H),(80,80,80),2)

        if writer: writer.write(canvas)
        if show:
            cv2.imshow("Lane", canvas)
            if cv2.waitKey(1)&0xFF==ord('q'): break
        if n%100==0:
            print(f"  Kare {n}: k={det.smooth_kayma:+.1f}  a={det.smooth_aci:+.1f}")

    cap.release()
    if writer: writer.release()
    if show: cv2.destroyAllWindows()
    print(f"Bitti – {n} kare  |  cikti: {output_path}")


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--test',   type=str, help='Test video yolu')
    ap.add_argument('--output', type=str, default=None)
    ap.add_argument('--show',   action='store_true')
    args = ap.parse_args()
    if args.test:
        main_test(args.test, args.output, args.show)
    else:
        main()

