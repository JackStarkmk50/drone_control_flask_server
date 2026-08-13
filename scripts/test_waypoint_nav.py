"""
Fly the real WaypointNav against NavSimSource. No Flask app, no vehicle, no
Pixhawk — runs anywhere, including on the Pi over SSH:

    python3 test_waypoint_nav.py

Covers what would otherwise only be discovered in the air: whether the loop
converges, whether the world-to-body rotation is correct at several headings,
whether the stick directions match DIR_MAP, and whether every safety gate
actually fires. Takes about two minutes.

Re-run after touching waypoint_nav.py, DIR_MAP, or LOIT_SPEED.
"""
import math, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from flight_tracker.sources import NavSimSource
import waypoint_nav as W
from waypoint_nav import WaypointNav

FAIL = []

def run_case(name, yaw_deg, wp, expect_reach=True, timeout=45.0):
    sim = NavSimSource(alt=1.5, yaw_deg=yaw_deg)
    sim.start()
    nav = WaypointNav(socketio=None, mode="rc")
    nav.attach(pose_fn=sim.pose, send_fn=sim.apply_rc)
    nav.start()

    # let it climb to hover
    t0 = time.time()
    while sim.pose()["z"] < 1.4 and time.time() - t0 < 15:
        time.sleep(0.1)

    r = nav.goto(*wp)
    if not r["ok"]:
        nav.stop(); sim.stop()
        print(f"  {name}: goto REJECTED — {r['message']}")
        return None

    peak = 0.0
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = sim.pose()
        d = math.hypot(wp[0] - p["x"], wp[1] - p["y"])
        peak = max(peak, math.hypot(p["x"], p["y"]))
        if nav.status()["state"] == "holding":
            break
        if nav.status()["state"] == "aborted":
            break
        time.sleep(0.1)

    st = nav.status()
    p = sim.pose()
    err = math.hypot(wp[0] - p["x"], wp[1] - p["y"])
    dt = time.time() - t0
    nav.stop(); sim.stop()

    ok = (st["state"] == "holding") and err <= W.ARRIVE_RADIUS_M
    tag = "PASS" if ok == expect_reach else "FAIL"
    if tag == "FAIL":
        FAIL.append(name)
    print(f"  [{tag}] {name}: yaw={yaw_deg:>3.0f}  target=({wp[0]},{wp[1]})  "
          f"final=({p['x']:+.2f},{p['y']:+.2f})  err={err:.3f}m  "
          f"t={dt:.1f}s  state={st['state']}  reason={st['reason']}")
    return p


print("\n=== 1. does it converge, and is the frame right? ===")
# yaw=0 (facing north): a pure-north waypoint must be pure PITCH.
run_case("north only", 0, (0.0, 4.0))
# yaw=0: a pure-east waypoint must be pure ROLL.
run_case("east only", 0, (4.0, 0.0))
run_case("diagonal", 0, (3.0, 4.0))
run_case("behind/left", 0, (-2.5, -3.5))

print("\n=== 2. body-frame rotation (this is where sign errors hide) ===")
# Facing east. A north waypoint is now to the aircraft's LEFT, so it must be
# flown with roll, not pitch. If the rotation is wrong it flies east instead.
run_case("yaw 90, go north", 90, (0.0, 4.0))
run_case("yaw 180, go north", 180, (0.0, 4.0))
run_case("yaw -90, go east", -90, (4.0, 0.0))
run_case("yaw 45, diagonal", 45, (3.0, 3.0))

print("\n=== 3. does the stick actually point the right way? ===")
sim = NavSimSource(alt=1.5, yaw_deg=0)
sim.start()
nav = WaypointNav(mode="rc")
nav.attach(pose_fn=sim.pose, send_fn=sim.apply_rc)
nav.start()
t0 = time.time()
while sim.pose()["z"] < 1.4 and time.time() - t0 < 15:
    time.sleep(0.1)

for label, wp, chan, want in (("north -> pitch below 1500", (0, 5), "pitch", "<"),
                              ("south -> pitch above 1500", (0, -5), "pitch", ">"),
                              ("east  -> roll  above 1500", (5, 0), "roll", ">"),
                              ("west  -> roll  below 1500", (-5, 0), "roll", "<")):
    nav.goto(*wp)
    time.sleep(1.2)
    v = sim._rc[chan]
    ok = (v < 1500) if want == "<" else (v > 1500)
    if not ok:
        FAIL.append(label)
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}: {chan}={v}")
    nav.abort("next case")
    sim._vn = sim._ve = 0.0
    sim._n = sim._e = 0.0
    time.sleep(0.4)
nav.stop(); sim.stop()

print("\n=== 4. safety gates ===")
sim = NavSimSource(alt=1.5)
sim.start()
nav = WaypointNav(mode="rc")
nav.attach(pose_fn=sim.pose, send_fn=sim.apply_rc)
nav.start()
time.sleep(4.0)

checks = [
    ("outside radius", lambda: nav.goto(20, 20), False),
    ("bad altitude",   lambda: nav.goto(1, 1, 9.0), False),
    ("non-numeric",    lambda: nav.goto("x", 1), False),
    ("valid",          lambda: nav.goto(1, 1), True),
]
for label, fn, want_ok in checks:
    r = fn()
    ok = r["ok"] == want_ok
    if not ok:
        FAIL.append(label)
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}: ok={r['ok']}  {r['message']}")

# disarm mid-leg must abort
nav.goto(4, 4)
time.sleep(0.6)
sim._armed = False
time.sleep(0.6)
st = nav.status()
ok = st["state"] == "aborted" and "armed" in (st["reason"] or "")
if not ok:
    FAIL.append("disarm aborts")
print(f"  [{'PASS' if ok else 'FAIL'}] disarm mid-leg aborts: "
      f"state={st['state']} reason={st['reason']}")
nav.stop(); sim.stop()

print("\n=== 5. dead reckoning must be refused ===")
class DrSim(NavSimSource):
    def pose(self):
        p = super().pose()
        p["src"] = "deadreckon"
        return p
sim = DrSim(alt=1.5); sim.start()
nav = WaypointNav(mode="rc")
nav.attach(pose_fn=sim.pose, send_fn=sim.apply_rc)
nav.start()
time.sleep(4.0)
r = nav.goto(2, 2)
ok = (not r["ok"]) and "dead reckoning" in r["message"]
if not ok:
    FAIL.append("deadreckon refused")
print(f"  [{'PASS' if ok else 'FAIL'}] {r['message']}")
nav.stop(); sim.stop()

print("\n=== 6. multi-waypoint queue ===")
sim = NavSimSource(alt=1.5); sim.start()
nav = WaypointNav(mode="rc")
nav.attach(pose_fn=sim.pose, send_fn=sim.apply_rc)
nav.start()
t0 = time.time()
while sim.pose()["z"] < 1.4 and time.time() - t0 < 15:
    time.sleep(0.1)
r = nav.queue([{"x": 3, "y": 0}, {"x": 3, "y": 3}, {"x": 0, "y": 3}, {"x": 0, "y": 0}])
print(f"  queued: {r['ok']} — {r['message']}")
t0 = time.time()
while time.time() - t0 < 90 and nav.status()["state"] != "holding":
    if nav.status()["state"] == "aborted":
        break
    time.sleep(0.2)
st = nav.status()
p = sim.pose()
back = math.hypot(p["x"], p["y"])
ok = st["reached"] == 4 and back <= W.ARRIVE_RADIUS_M
if not ok:
    FAIL.append("square queue")
print(f"  [{'PASS' if ok else 'FAIL'}] 3m square: reached={st['reached']}/4  "
      f"closed at ({p['x']:+.2f},{p['y']:+.2f}) = {back:.3f}m from origin  "
      f"t={time.time()-t0:.1f}s  state={st['state']}")
nav.stop(); sim.stop()

print("\n" + "=" * 60)
if FAIL:
    print(f"{len(FAIL)} FAILURES: {FAIL}")
    sys.exit(1)
print("ALL PASS")
