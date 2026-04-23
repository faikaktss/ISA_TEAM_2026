"""
Hafif performans ölçüm modülü.
Terminal'e hiçbir şey yazmaz — /perf/summary topic'i dinle:
    ros2 topic echo /perf/summary
"""
import time
import threading
import collections


class FPSMeter:
    """Sliding-window FPS hesaplayıcı."""

    def __init__(self, name: str, window: int = 60):
        self.name = name
        self._times: collections.deque = collections.deque(maxlen=window)
        self._lock = threading.Lock()

    def tick(self) -> float:
        now = time.perf_counter()
        with self._lock:
            self._times.append(now)
            n = len(self._times)
            if n < 2:
                return 0.0
            return (n - 1) / (self._times[-1] - self._times[0])

    def fps(self) -> float:
        with self._lock:
            n = len(self._times)
            if n < 2:
                return 0.0
            return (n - 1) / (self._times[-1] - self._times[0])


class CallbackTimer:
    """Callback süresini ve bütçe ihlallerini ölçer."""

    def __init__(self, name: str, budget_ms: float = 33.0):
        self.name = name
        self.budget_ms = budget_ms
        self._last_ms = 0.0
        self._violations = 0
        self._total_calls = 0
        self._lock = threading.Lock()
        self._t0 = None

    def start(self):
        self._t0 = time.perf_counter()

    def stop(self) -> float:
        if self._t0 is None:
            return 0.0
        elapsed_ms = (time.perf_counter() - self._t0) * 1000.0
        with self._lock:
            self._last_ms = elapsed_ms
            self._total_calls += 1
            if elapsed_ms > self.budget_ms:
                self._violations += 1
        self._t0 = None
        return elapsed_ms

    def summary(self) -> str:
        with self._lock:
            return (f"{self.name}={self._last_ms:.1f}ms "
                    f"viol={self._violations}/{self._total_calls}")


class PerfPublisher:
    """
    Her `interval` saniyede bir /perf/summary topic'ine String publish eder.
    Terminal'e hiçbir şey yazmaz.

    Kullanım:
        self._perf = PerfPublisher(self, interval=2.0)
        self._perf.add_fps('zed_cap', self._zed_cap_fps)
        self._perf.add_timer('publish_cb', self._publish_timer)
        # hot path'de:
        self._perf.tick()
    """

    def __init__(self, node, interval: float = 2.0):
        from std_msgs.msg import String
        self._pub = node.create_publisher(String, '/perf/summary', 10)
        self._interval = interval
        self._last = time.perf_counter()
        self._fps_meters: dict = {}
        self._cb_timers: dict = {}
        self._value_suppliers: dict = {}
        self._String = String

    def add_fps(self, key: str, meter: FPSMeter):
        self._fps_meters[key] = meter

    def add_timer(self, key: str, timer: CallbackTimer):
        self._cb_timers[key] = timer

    def add_value(self, key: str, supplier):
        self._value_suppliers[key] = supplier

    def tick(self):
        now = time.perf_counter()
        if now - self._last < self._interval:
            return
        self._last = now

        parts = []
        for key, m in self._fps_meters.items():
            parts.append(f"{key}={m.fps():.1f}fps")
        for key, t in self._cb_timers.items():
            parts.append(t.summary())
        for key, supplier in self._value_suppliers.items():
            try:
                value = supplier()
            except Exception:
                continue
            if value is not None:
                parts.append(f"{key}={value}")

        if parts:
            msg = self._String()
            msg.data = " | ".join(parts)
            self._pub.publish(msg)