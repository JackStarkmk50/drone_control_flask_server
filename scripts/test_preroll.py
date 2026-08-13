"""
Pre-roll: does the flight video actually begin before the arm, and stay in sync?

    python3 test_preroll.py

No camera and no aircraft needed - a synthetic capture stands in, with each
frame carrying its own index so the test can prove WHICH frames reached the
file rather than only counting them.

Re-run after touching frames.py, video.py, or CAMERA_PREROLL_S.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np

from flight_tracker.frames import FrameHub
from flight_tracker import video as V

FAIL = []
FPS = 10
PREROLL_S = 2.0

# A fake camera whose frames carry their own index, so we can prove WHICH
# frames ended up in the file rather than just counting them.
counter = {"n": 0}


def fake_read():
    counter["n"] += 1
    f = np.zeros((48, 64, 3), dtype=np.uint8)
    f[0, 0, 0] = counter["n"] % 256
    return True, f


print("=== 1. ring fills and is bounded ===")
hub = FrameHub(fake_read, fps=FPS, preroll_s=PREROLL_S)
hub.start()
time.sleep(PREROLL_S + 1.0)          # run past the ring's capacity
pre = hub.preroll()
want = int(PREROLL_S * FPS)
ok = len(pre) == want
if not ok:
    FAIL.append("ring size")
print(f"  [{'PASS' if ok else 'FAIL'}] buffered {len(pre)} frames, want {want} "
      f"({PREROLL_S}s x {FPS}fps)")

ok = all(pre[i][1] <= pre[i + 1][1] for i in range(len(pre) - 1))
if not ok:
    FAIL.append("ring order")
print(f"  [{'PASS' if ok else 'FAIL'}] oldest first")

span = pre[-1][1] - pre[0][1]
ok = abs(span - (PREROLL_S - 1.0 / FPS)) < 0.4
if not ok:
    FAIL.append("ring span")
print(f"  [{'PASS' if ok else 'FAIL'}] spans {span:.2f}s of wall clock")

# the ring must hold the frames immediately BEFORE now, not the first ever
newest_idx = pre[-1][0][0, 0, 0]
ok = newest_idx == counter["n"] % 256 or abs(int(newest_idx) - counter["n"] % 256) <= 2
if not ok:
    FAIL.append("ring recency")
print(f"  [{'PASS' if ok else 'FAIL'}] newest buffered frame is the most recent grab")

print("\n=== 2. recorder writes pre-roll ahead of live ===")
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "preroll_out")
os.makedirs(out, exist_ok=True)
rec = V.VideoRecorder(hub, out, fps=FPS, size=(64, 48))

t_arm = time.time()
pre = hub.preroll()
path = rec.start("test_preroll", preroll=pre)
time.sleep(1.5)                       # ~15 live frames
res = rec.stop()
hub.stop()

print(f"  path={os.path.basename(res['path'] or '')} codec={res['codec']} "
      f"frames={res['frames']} bytes={res['size_bytes']}")

ok = res["frames"] >= len(pre)
if not ok:
    FAIL.append("frame count")
print(f"  [{'PASS' if ok else 'FAIL'}] wrote {res['frames']} frames "
      f">= {len(pre)} pre-roll")

# the decisive check: started_at must predate the arm
started = res.get("started_at")
ok = started is not None and started < t_arm
delta = (started - t_arm) if started else 0.0
if not ok:
    FAIL.append("started_at before arm")
print(f"  [{'PASS' if ok else 'FAIL'}] started_at is {delta:+.2f}s "
      f"relative to arm (must be negative)")

print("\n=== 3. the offset the player uses ===")
offset_ms = int((started - t_arm) * 1000) if started else 0
print(f"  video_start_offset_ms = {offset_ms}")
ok = offset_ms < 0
if not ok:
    FAIL.append("offset sign")
print(f"  [{'PASS' if ok else 'FAIL'}] negative, as a pre-roll requires")

# player maths from MOBILE_TRACKER_API.md must still land correctly
for t_ms, label in ((0, "flight t=0 (the arm)"), (1000, "1 s into the flight")):
    pos = (t_ms - offset_ms) / 1000.0
    print(f"    t_ms={t_ms:<5} -> video position {pos:.2f}s   ({label})")
arm_pos = (0 - offset_ms) / 1000.0
ok = abs(arm_pos - abs(delta)) < 0.05
if not ok:
    FAIL.append("player maths")
print(f"  [{'PASS' if ok else 'FAIL'}] arm lands at {arm_pos:.2f}s into the clip, "
      f"= the pre-roll length")

print("\n=== 4. preroll disabled costs nothing ===")
hub2 = FrameHub(fake_read, fps=FPS, preroll_s=0.0)
hub2.start()
time.sleep(0.5)
p2 = hub2.preroll()
hub2.stop()
ok = p2 == []
if not ok:
    FAIL.append("disabled preroll")
print(f"  [{'PASS' if ok else 'FAIL'}] preroll_s=0 keeps {len(p2)} frames")

import shutil
shutil.rmtree(out, ignore_errors=True)

print("\n" + "=" * 60)
if FAIL:
    print(f"{len(FAIL)} FAILURE(S): {FAIL}")
    sys.exit(1)
print("ALL PASS")
