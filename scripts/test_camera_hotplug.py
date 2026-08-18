"""
Hot-plug: server starts with NO camera, camera is connected later, no restart.

    python3 test_camera_hotplug.py

Drives the REAL start_camera()/stop_camera() out of new_api_server_1 with
cv2.VideoCapture swapped for a fake device that can be plugged and unplugged,
so the whole recovery path runs without hardware.
"""
import os
import shutil
import sys
import threading
import time
import types

SCRIPTS = os.path.dirname(os.path.abspath(__file__))
FLIGHTS = os.path.join(os.path.dirname(SCRIPTS), "flights")
had_flights = os.path.isdir(FLIGHTS)
sys.path.insert(0, SCRIPTS)

# dronekit/pymavlink are not needed here; stub them if absent (dev box).
try:
    import dronekit, pymavlink.mavutil          # noqa: F401
except ImportError:
    class _Any:
        def __init__(self, *a, **kw): pass
        def __getattr__(self, k):
            if k.startswith("__"): raise AttributeError(k)
            return _Any()
        def __call__(self, *a, **kw): return _Any()
    dk = types.ModuleType("dronekit")
    dk.connect = lambda *a, **kw: _Any()
    dk.VehicleMode = lambda n="": _Any()
    dk.Vehicle = dk.LocationGlobalRelative = dk.LocationLocal = _Any
    sys.modules["dronekit"] = dk
    pm = types.ModuleType("pymavlink")
    mv = types.ModuleType("pymavlink.mavutil")
    class _M:
        def __getattr__(self, k): return 0
    mv.mavlink = _M()
    mv.mavlink_connection = lambda *a, **kw: _Any()
    pm.mavutil = mv
    sys.modules["pymavlink"], sys.modules["pymavlink.mavutil"] = pm, mv

import new_api_server_1 as S

fails = []


def check(label, ok, detail=""):
    if not ok:
        fails.append(label)
    print(f"  [{'PASS' if ok else 'FAIL'}] {label:<52} {detail}")


# ── the fake device ──────────────────────────────────────────────────
world = {"plugged": False}
opened = []          # every capture handed out, so we can prove they get released


class FakeCap:
    def __init__(self, *a, **kw):
        self.live = world["plugged"]
        self.released = False
        self.n = 0
        opened.append(self)

    def set(self, *a):        return True
    def isOpened(self):       return self.live and not self.released

    def grab(self):
        if not self.isOpened() or not world["plugged"]:
            return False
        time.sleep(0.006)     # slower than DRAIN_FRESH_S: reads as a live frame
        self.n += 1
        return True

    def retrieve(self):
        return (True, ("frame", self.n)) if self.isOpened() else (False, None)

    def read(self):
        return (True, ("frame", self.n)) if self.grab() else (False, None)

    def release(self):
        self.released = True


class _FakeBuf:
    def tobytes(self):
        return bytes([0xFF, 0xD8]) + b"j" * 400 + bytes([0xFF, 0xD9])  # JPEG-shaped


S.cv2 = types.SimpleNamespace(
    VideoCapture=lambda *a, **kw: FakeCap(),
    CAP_V4L2=0, CAP_PROP_BUFFERSIZE=0, CAP_PROP_FOURCC=0,
    CAP_PROP_FRAME_WIDTH=0, CAP_PROP_FRAME_HEIGHT=0, CAP_PROP_FPS=0,
    VideoWriter_fourcc=lambda *a: 0,
    IMWRITE_JPEG_QUALITY=1,
    imencode=lambda ext, frame, params=None: (True, _FakeBuf()),
)
S.time.sleep = time.sleep     # keep the 1 s settle honest but bounded


def reset():
    S.camera, S.camera_active = None, False


print("=" * 74)
print("1. SERVER BOOTS WITH NO CAMERA")
print("=" * 74)
reset()
world["plugged"] = False
try:
    S.start_camera()
    check("start_camera() raises when the device is absent", False, "it returned")
except RuntimeError as e:
    check("start_camera() raises when the device is absent", True, f"{e}")
check("no capture left behind", S.camera is None and not S.camera_active)
check("every failed capture was released", all(c.released for c in opened),
      f"{len(opened)} opened")
check("grabber not running", not S.frame_hub.running)

print()
print("=" * 74)
print("2. /camera/start AND /camera/stream ANSWER HONESTLY (no silent 200)")
print("=" * 74)
c = S.app.test_client()
r = c.post("/camera/start")
check("POST /camera/start -> 500", r.status_code == 500, r.status_code)
check("  body says success:false", r.get_json().get("success") is False,
      r.get_json().get("message"))
r = c.get("/camera/stream")
check("GET /camera/stream -> 503, not an empty 200", r.status_code == 503,
      r.status_code)
check("  body is JSON the UI can show",
      "Camera unavailable" in r.get_json().get("message", ""),
      r.get_json().get("message"))

print()
print("=" * 74)
print("3. CAMERA PLUGGED IN MID-RUN — NO SERVER RESTART")
print("=" * 74)
world["plugged"] = True
r = c.post("/camera/start")
check("POST /camera/start -> 200", r.status_code == 200, r.status_code)
check("  capture is open", S.camera is not None and S.camera_active)
check("  grabber started", S.frame_hub.running)
time.sleep(0.3)
seq, frame, _ = S.frame_hub.latest()
check("  frames are flowing", frame is not None, f"seq={seq}")

body = next(S.generate_frames())
check("generate_frames() yields a real JPEG part",
      body.startswith(b"--frame") and len(body) > 100, f"{len(body)} bytes")

print()
print("=" * 74)
print("4. CAMERA UNPLUGGED MID-RUN, THEN PLUGGED BACK IN")
print("=" * 74)
world["plugged"] = False
# A brief glitch is survivable on purpose: the grabber tolerates 20 consecutive
# failures, and at fps=10 that is a full 2 s of dead device before it gives up.
time.sleep(0.6)
check("a short glitch does NOT kill the grabber", S.frame_hub.running)
world["plugged"] = True
time.sleep(0.3)
check("  and it recovers on its own, no user action", S.frame_hub.running)

world["plugged"] = False
time.sleep(2.6)                       # now past the 20-failure budget
check("a long outage does kill the grabber", not S.frame_hub.running)
check("but the stale handle is still held", S.camera is not None)

world["plugged"] = True
before = len(opened)
r = c.post("/camera/start")           # user just clicks start again
check("POST /camera/start -> 200 without a restart", r.status_code == 200,
      r.status_code)
check("  the stale handle was dropped and reopened",
      len(opened) == before + 1 and opened[before - 1].released)
check("  grabber running again", S.frame_hub.running)
time.sleep(0.3)
seq2, frame2, _ = S.frame_hub.latest()
check("  frames flowing again", frame2 is not None, f"seq={seq2}")

print()
print("=" * 74)
print("5. STOP RELEASES CLEANLY")
print("=" * 74)
check("stop_camera() returns True", S.stop_camera() is True)
check("  capture released", S.camera is None and not S.camera_active)
check("  grabber joined, no thread left",
      not S.frame_hub.running and
      not any(t.name == "framehub" for t in threading.enumerate()))
check("  and it can start again", (S.start_camera(), S.camera is not None)[1])
S.stop_camera()

print()
print("=" * 74)
if fails:
    print(f"{len(fails)} FAILURE(S)")
    for f in fails:
        print("  -", f)
else:
    print("ALL PASS")

try:
    S.tracker.stop()
except Exception:
    pass
if not had_flights and os.path.isdir(FLIGHTS):
    shutil.rmtree(FLIGHTS, ignore_errors=True)
sys.exit(1 if fails else 0)
