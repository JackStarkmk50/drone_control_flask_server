"""
The camera bugs behind "start does nothing, then it stops itself" and
"the stream lags".

  python3 test_camera_stream.py

No camera, no server. A fake capture stands in for cv2.VideoCapture so the
V4L2 queue behaviour can be reproduced exactly.
"""
import sys
import threading
import time

sys.path.insert(0, __import__("os").path.dirname(__file__))
from flight_tracker.frames import FrameHub

fails = []


def check(label, ok, detail=""):
    if not ok:
        fails.append(label)
    print(f"  [{'PASS' if ok else 'FAIL'}] {label:<50} {detail}")


# ── 1. stop() must join, or a stop/start pair runs two grabbers ──────
print("=" * 70)
print("1. stop()/start() DOES NOT LEAVE A SECOND GRABBER RUNNING")
print("=" * 70)

live = []          # thread idents currently inside the read fn


def slow_read():
    live.append(threading.current_thread().ident)
    time.sleep(0.08)                       # a read() that blocks, like V4L2
    live.pop()
    return True, object()


hub = FrameHub(slow_read, fps=10, name="t1")
hub.start()
time.sleep(0.15)
peak = 0
for _ in range(6):
    hub.stop()                              # what the UI toggle does
    hub.start()
    time.sleep(0.05)
    peak = max(peak, len(live))
hub.stop()

check("only ever one thread inside read()", peak <= 1, f"peak={peak}")
check("stop() leaves the grabber dead", not hub.running)
check("no grabber thread survives stop()",
      not any(t.name == "t1" for t in threading.enumerate()))

# ── 2. the grabber restarts after it dies ───────────────────────────
print()
print("=" * 70)
print("2. A DEAD GRABBER RESTARTS (no process restart needed)")
print("=" * 70)

state = {"ok": False}


def flaky_read():
    time.sleep(0.005)
    return (True, object()) if state["ok"] else (False, None)


hub2 = FrameHub(flaky_read, fps=50, name="t2")
hub2.start()
time.sleep(0.6)                             # 20 misses -> the loop breaks
check("grabber gives up after 20 failures", not hub2.running)

state["ok"] = True
hub2.start()                                # exactly what start_camera() does
time.sleep(0.1)
seq, frame, _ = hub2.latest()
check("start() revives it", hub2.running)
check("frames flow again", frame is not None, f"seq={seq}")
hub2.stop()

# ── 3. the drain: newest frame, one decode ──────────────────────────
print()
print("=" * 70)
print("3. V4L2 DRAIN — READ THE NEWEST FRAME, NOT THE OLDEST QUEUED")
print("=" * 70)


class FakeV4L2:
    """
    A UVC camera that ignored CAP_PROP_FPS: it produces at 30 fps into a
    4-deep kernel queue while the consumer asks for 10. grab() pops the
    queue and only blocks when the queue is empty; retrieve() decodes.
    """

    def __init__(self, depth=4, period=1 / 30.0):
        self.depth, self.period = depth, period
        self.produced = 0
        self.t = time.time()
        self.q = []
        self.decodes = 0

    def _fill(self):
        now = time.time()
        while now - self.t >= self.period:
            self.t += self.period
            self.produced += 1
            self.q.append(self.produced)
            if len(self.q) > self.depth:
                self.q.pop(0)               # driver drops the oldest

    def grab(self):
        self._fill()
        if not self.q:                      # empty: wait for the sensor
            time.sleep(self.period)
            self._fill()
        self.cur = self.q.pop(0)
        return True

    def retrieve(self):
        self.decodes += 1
        return True, self.cur

    def read(self):
        self.grab()
        return self.retrieve()


DRAIN_MAX, DRAIN_FRESH_S = 8, 0.005


def drained(cap):
    for _ in range(DRAIN_MAX):
        t0 = time.time()
        if not cap.grab():
            return False, None
        if time.time() - t0 > DRAIN_FRESH_S:
            break
    return cap.retrieve()


for label, fn in (("plain read()", lambda c: c.read()), ("drained", drained)):
    cap = FakeV4L2()
    time.sleep(0.2)                         # let a backlog build, as at startup
    lags = []
    for _ in range(20):
        ok, got = fn(cap)
        cap._fill()
        lags.append(cap.produced - got)     # frames behind live
        time.sleep(0.1)                     # consumer at 10 fps
    lag = sum(lags) / len(lags)
    ms = lag * 1000 / 30.0
    print(f"  {label:<14} mean lag {lag:5.1f} frames ({ms:5.0f} ms)   "
          f"{cap.decodes} decodes")
    if label == "plain read()":
        base = lag
        check("plain read() lags behind live", lag >= 2.0, f"{lag:.1f} frames")
    else:
        check("drain cuts the lag", lag < 1.5, f"{lag:.1f} vs {base:.1f} frames")
        check("drain still decodes once per cycle", cap.decodes == 20,
              f"{cap.decodes} decodes for 20 cycles")

# a camera that DOES honour the requested rate must cost nothing extra
cap = FakeV4L2(depth=4, period=0.1)
for _ in range(10):
    drained(cap)
check("no cost when the camera is already at rate", cap.decodes == 10,
      f"{cap.decodes} decodes for 10 cycles")

print()
print("=" * 70)
if fails:
    print(f"{len(fails)} FAILURE(S)")
    for f in fails:
        print("  -", f)
else:
    print("ALL PASS")
sys.exit(1 if fails else 0)
