"""
Actually import new_api_server_1.py and check the wiring came out live.

    python3 verify_flask.py

Uses the real dronekit and pymavlink when they are installed (i.e. on the Pi)
and stubs them when they are not, so the same script works on a dev box.

Until now that file has only been AST-parsed. Parsing proves the syntax is
valid; it does not prove a name exists. Importing executes every module-level
statement, which is where a constant used before it is defined, a typo in a
helper name, or a bad decorator actually shows up.
"""
import os
import shutil
import sys
import types

SCRIPTS = os.path.dirname(os.path.abspath(__file__))
FLIGHTS = os.path.join(os.path.dirname(SCRIPTS), "flights")
had_flights = os.path.isdir(FLIGHTS)


# ── stub dronekit / pymavlink ────────────────────────────────────────
class _Any:
    def __init__(self, *a, **kw):
        object.__setattr__(self, "_d", dict(kw))

    def __getattr__(self, k):
        if k.startswith("__"):
            raise AttributeError(k)
        d = object.__getattribute__(self, "_d")
        if k not in d:
            d[k] = _Any()
        return d[k]

    def __setattr__(self, k, v):
        object.__getattribute__(self, "_d")[k] = v

    def __call__(self, *a, **kw):
        return _Any()


# Stub ONLY what is missing. On the Pi, dronekit and pymavlink are really
# installed and must not be shadowed — the point is to import the server the
# way it will actually be imported there.
try:
    import dronekit  # noqa: F401
    import pymavlink.mavutil  # noqa: F401
    print("(using the real dronekit / pymavlink)")
    _REAL = True
except ImportError:
    print("(dronekit/pymavlink absent - stubbing them)")
    _REAL = False

dk = types.ModuleType("dronekit")
dk.connect = lambda *a, **kw: _Any()
dk.VehicleMode = lambda n="": _Any(name=n)
dk.Vehicle = _Any
dk.LocationGlobalRelative = _Any
dk.LocationLocal = _Any
if not _REAL:
    sys.modules["dronekit"] = dk

pm = types.ModuleType("pymavlink")
mavutil = types.ModuleType("pymavlink.mavutil")


class _Mavlink:
    def __getattr__(self, k):
        return 0


mavutil.mavlink = _Mavlink()
mavutil.mavlink_connection = lambda *a, **kw: _Any()
pm.mavutil = mavutil
if not _REAL:
    sys.modules["pymavlink"] = pm
    sys.modules["pymavlink.mavutil"] = mavutil

sys.path.insert(0, SCRIPTS)

fails = []

print("=" * 68)
print("1. IMPORT new_api_server_1  (executes every module-level statement)")
print("=" * 68)
try:
    import new_api_server_1 as S
    print("  ok       imported")
except Exception as e:
    print(f"  IMPORT!  {type(e).__name__}: {e}")
    import traceback
    traceback.print_exc(limit=8)
    sys.exit(1)

print()
print("=" * 68)
print("2. THE PRE-ROLL / CAMERA EDITS ARE LIVE")
print("=" * 68)
checks = [
    ("CAMERA_PREROLL_S defined", hasattr(S, "CAMERA_PREROLL_S"),
     getattr(S, "CAMERA_PREROLL_S", None)),
    ("CAMERA_AUTOSTART defined", hasattr(S, "CAMERA_AUTOSTART"),
     getattr(S, "CAMERA_AUTOSTART", None)),
    ("frame_hub constructed", S.frame_hub is not None, type(S.frame_hub).__name__),
    ("hub got the preroll seconds",
     getattr(S.frame_hub, "preroll_s", None) == S.CAMERA_PREROLL_S,
     getattr(S.frame_hub, "preroll_s", None)),
    ("hub ring sized from fps",
     S.frame_hub._ring.maxlen == int(round(S.CAMERA_PREROLL_S * S.CAMERA_FPS)),
     S.frame_hub._ring.maxlen),
    ("hub exposes preroll()", callable(getattr(S.frame_hub, "preroll", None)), ""),
    ("tracker constructed", S.tracker is not None, type(S.tracker).__name__),
    ("nav constructed", S.nav is not None, type(S.nav).__name__),
    ("start_camera exists for autostart", callable(getattr(S, "start_camera", None)), ""),
]
for label, ok, detail in checks:
    if not ok:
        fails.append(label)
    print(f"  [{'PASS' if ok else 'FAIL'}] {label:<38} {detail}")

print()
print("=" * 68)
print("3. VideoRecorder.start ACCEPTS preroll")
print("=" * 68)
import inspect
from flight_tracker.video import VideoRecorder
sig = inspect.signature(VideoRecorder.start)
ok = "preroll" in sig.parameters
if not ok:
    fails.append("VideoRecorder.start(preroll=)")
print(f"  [{'PASS' if ok else 'FAIL'}] VideoRecorder.start{sig}")

src = inspect.getsource(S.tracker._begin_flight if hasattr(S.tracker, "_begin_flight")
                        else type(S.tracker)._begin_flight)
ok = "preroll=" in src and ".preroll()" in src
if not ok:
    fails.append("_begin_flight passes preroll")
print(f"  [{'PASS' if ok else 'FAIL'}] _begin_flight pulls hub.preroll() and passes it on")

print()
print("=" * 68)
print("4. FLASK ROUTES REGISTERED  (the whole HTTP contract, live)")
print("=" * 68)
rules = sorted({r.rule for r in S.app.url_map.iter_rules()})
expect = ["/", "/health", "/status", "/arm", "/disarm", "/safety", "/takeoff",
          "/land", "/rtl", "/hold", "/emergency", "/mode", "/lvlcal", "/move",
          "/yaw", "/rc", "/param", "/pid/all", "/pid/set", "/pid/reset",
          "/pid/save", "/pid/files", "/pid/load", "/mission", "/mission/status",
          "/mission/cancel", "/queue/add", "/queue/status", "/queue/clear",
          "/camera/stream", "/camera/start", "/camera/stop", "/network",
          "/network/scan", "/network/connect", "/wifi/scan", "/wifi/connect",
          "/flights/", "/flights/current", "/flights/stats",
          "/flights/sim/start", "/flights/sim/stop",
          "/nav/goto", "/nav/queue", "/nav/status", "/nav/abort", "/nav/clear"]
missing = [e for e in expect if e not in rules]
print(f"  {len(expect) - len(missing)}/{len(expect)} expected rules present "
      f"({len(rules)} total registered)")
for m in missing:
    fails.append(f"route missing: {m}")
    print(f"    MISSING  {m}")

print()
print("=" * 68)
print("5. NAV ABORT HOOKS STILL WIRED")
print("=" * 68)
srv_src = open(os.path.join(SCRIPTS, "new_api_server_1.py"), encoding="utf-8").read()
n = srv_src.count("_nav_abort(")
ok = n >= 10          # 1 definition + 9 call sites
if not ok:
    fails.append("nav abort hooks")
print(f"  [{'PASS' if ok else 'FAIL'}] {n - 1} call sites + 1 definition")

print()
print("=" * 68)
if fails:
    print(f"{len(fails)} FAILURE(S)")
    for f in fails:
        print("  -", f)
else:
    print("no failures")

# clean up anything the import created
try:
    S.tracker.stop()
except Exception:
    pass
if not had_flights and os.path.isdir(FLIGHTS):
    shutil.rmtree(FLIGHTS, ignore_errors=True)
    print("\n(removed the flights/ dir this import created)")

sys.exit(1 if fails else 0)
