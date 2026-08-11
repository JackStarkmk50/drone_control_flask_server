"""
Flight recorder.

One background thread. It polls a telemetry source at SAMPLE_HZ, detects the
arm/disarm edges, and turns each armed period into one flight row plus a track.

Design rules this file follows:

  * PASSIVE. The recorder reads the vehicle and the MovementController. It
    never writes to either. A bug in here cannot move the aircraft.
  * NEVER BLOCKS THE FLIGHT. Every DB write is batched and every exception is
    swallowed with a log. Losing telemetry history is acceptable; stalling a
    thread that shares a process with the 20 Hz RC override is not.
  * ORIGIN-RELATIVE. local_frame is measured from the EKF origin, which is set
    once when the EKF initialises and then PERSISTS across arm/disarm cycles in
    the same power session. So the raw reading at the second arm of a session
    is not zero. The reading at arm is stored as flights.origin_* and
    subtracted from every point, which is what makes each arm an independent
    track starting at (0,0,0) regardless of what came before it.
"""

import os
import threading
import time
from datetime import datetime

from .sources import LiveSource, SimSource
from .store import FlightStore

SAMPLE_HZ = 5.0
SAMPLE_PERIOD = 1.0 / SAMPLE_HZ

# Rows are buffered and written in batches. At 5 Hz that is ~5 rows per commit,
# which keeps SD-card write amplification down without risking much data.
FLUSH_PERIOD_S = 1.0

# How often a live point is pushed to connected clients. Same as the sample
# rate by default; raise the divisor if mobile clients struggle.
EMIT_EVERY_N = 1

# Poll interval while disarmed. No reason to sample at 5 Hz on the bench.
IDLE_PERIOD_S = 0.5


def _sub(a, b):
    """a - b, tolerating None on either side."""
    if a is None or b is None:
        return None
    return a - b


class FlightTracker:
    """
    Facade the server holds. Owns the store, the recorder thread and the
    optional video recorder.
    """

    def __init__(self, db_path, video_dir, socketio=None,
                 record_video=True, video_fps=10, video_size=(640, 480),
                 video_bitrate="800k"):
        self.store = FlightStore(db_path)
        self.video_dir = os.path.abspath(video_dir)
        os.makedirs(self.video_dir, exist_ok=True)
        self.socketio = socketio

        self.record_video = record_video
        self._video_cfg = dict(fps=video_fps, size=video_size, bitrate=video_bitrate)

        self._source = None
        self._sim = None
        self._hub = None
        self._camera_start = None
        self._video = None

        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        # Live flight state
        self.flight_id = None
        self.flight_uuid = None
        self._t0 = None
        self._buf = []
        self._last_flush = 0.0
        self._origin = (None, None, None)
        self._summary = {}
        self._prev_xyz = None
        self._emit_n = 0
        self._pos_src_count = {}
        self._last_mode = None

        orphans = self.store.close_orphans()
        if orphans:
            print(f"[tracker] closed {len(orphans)} interrupted flight(s): {orphans}")

    # ── wiring ────────────────────────────────────────────────────────

    def attach(self, vehicle, mc=None, frame_hub=None, camera_start=None):
        """Called by the server once the vehicle is connected."""
        self._source = LiveSource(vehicle, mc)
        self._hub = frame_hub
        self._camera_start = camera_start
        if self.record_video and frame_hub is not None:
            from .video import VideoRecorder
            self._video = VideoRecorder(frame_hub, self.video_dir, **self._video_cfg)
        print(f"[tracker] attached  db={self.store.db_path}  video={self.video_dir}")

    def start(self):
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, daemon=True,
                                            name="flight-recorder")
            self._thread.start()
        print(f"[tracker] recorder thread started @ {SAMPLE_HZ:g} Hz")

    def stop(self):
        self._stop.set()
        t = self._thread
        if t:
            t.join(timeout=5.0)
        if self.flight_id is not None:
            self._end_flight("manual")
        self.store.close()

    # ── sim control ───────────────────────────────────────────────────

    def start_sim(self, **kw):
        """
        Run a synthetic flight. Lets the whole UI be built and demoed with no
        Pixhawk attached — the DB rows, the socket events and the REST payloads
        are identical to a real flight.
        """
        with self._lock:
            if self.flight_id is not None:
                return {"success": False, "message": "a flight is already recording"}
            sim = SimSource(**kw)
            sim.start()
            self._sim = sim
        # start() takes the same lock, and threading.Lock is not reentrant —
        # calling it inside the block above deadlocks the request thread.
        self.start()
        return {"success": True, "message": "sim flight started",
                "duration_s": round(sim.total_s, 1)}

    def stop_sim(self):
        if self._sim:
            self._sim.stop()
            return {"success": True}
        return {"success": False, "message": "no sim running"}

    # ── main loop ─────────────────────────────────────────────────────

    def _active_source(self):
        if self._sim is not None:
            # Keep returning the sim while its flight is still open, even after
            # the profile finishes — the loop needs one more pass to observe
            # is_armed()==False and close the flight. Dropping the source the
            # instant the profile ended left the flight open forever.
            if not self._sim.is_done() or self.flight_id is not None:
                return self._sim
            self._sim = None
        return self._source

    def _loop(self):
        next_t = time.time()
        while not self._stop.is_set():
            try:
                src = self._active_source()
                if src is None:
                    time.sleep(IDLE_PERIOD_S)
                    next_t = time.time()
                    continue

                armed = src.is_armed()

                if armed and self.flight_id is None:
                    self._begin_flight(src)
                elif not armed and self.flight_id is not None:
                    self._end_flight("disarm")

                if self.flight_id is not None:
                    self._sample(src)
                    period = SAMPLE_PERIOD
                else:
                    period = IDLE_PERIOD_S

            except Exception as e:
                print(f"[tracker] loop error: {type(e).__name__}: {e}")
                period = IDLE_PERIOD_S

            next_t += period
            slack = next_t - time.time()
            if slack > 0:
                time.sleep(slack)
            else:
                next_t = time.time()

        # Thread exiting with a flight open (process shutdown).
        if self.flight_id is not None:
            self._end_flight("interrupted")

    # ── flight lifecycle ──────────────────────────────────────────────

    def _begin_flight(self, src):
        s = src.sample()
        origin = (s.get("north"), s.get("east"), s.get("down"))

        try:
            src.reset()
        except Exception:
            pass

        fid, fuuid = self.store.open_flight(source=getattr(src, "name", "live"),
                                            origin=origin)
        self.flight_id = fid
        self.flight_uuid = fuuid
        self._t0 = time.time()
        self._origin = origin
        self._buf = []
        self._last_flush = self._t0
        self._prev_xyz = None
        self._summary = {
            "point_count": 0, "max_alt_m": 0.0, "max_dist_m": 0.0,
            "distance_m": 0.0, "max_groundspeed": 0.0,
            "battery_start_v": s.get("battery_v"), "battery_end_v": s.get("battery_v"),
        }
        self._pos_src_count = {}
        self._last_mode = s.get("mode")

        self.store.insert_event(fid, 0, "arm", s.get("mode"))

        vpath = None
        if self._video is not None:
            try:
                if self._camera_start:
                    self._camera_start()
                if self._hub is not None and not self._hub.running:
                    self._hub.start()
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                vpath = self._video.start(f"flight_{fid:05d}_{stamp}")
            except Exception as e:
                print(f"[tracker] video start failed: {type(e).__name__}: {e}")

        print(f"[tracker] FLIGHT {fid} started  origin={origin}  video={vpath}")
        self._emit("flight_started", {
            "id": fid, "uuid": fuuid, "source": getattr(src, "name", "live"),
            "started_at": time.time(), "video": bool(vpath),
        })

    def _end_flight(self, reason):
        fid = self.flight_id
        if fid is None:
            return
        self._flush(force=True)

        dur = time.time() - (self._t0 or time.time())
        self._summary["duration_s"] = round(dur, 2)
        if self._pos_src_count:
            self._summary["pos_source"] = max(self._pos_src_count,
                                              key=self._pos_src_count.get)
        for k in ("max_alt_m", "max_dist_m", "distance_m", "max_groundspeed"):
            if self._summary.get(k) is not None:
                self._summary[k] = round(self._summary[k], 3)

        self.store.insert_event(fid, int(dur * 1000), "disarm", reason)

        if self._video is not None and self._video.active:
            try:
                res = self._video.stop()
                offset_ms = None
                if res.get("started_at") and self._t0:
                    offset_ms = int((res["started_at"] - self._t0) * 1000)
                if res.get("path") and res.get("frames"):
                    self.store.set_video(fid, res["path"], res["codec"], offset_ms,
                                         res["duration_s"], res["size_bytes"])
                    print(f"[tracker] video saved {res['path']} "
                          f"({res['frames']} frames, {res['size_bytes']} bytes)")
            except Exception as e:
                print(f"[tracker] video stop failed: {type(e).__name__}: {e}")

        self.store.close_flight(fid, reason, self._summary)

        print(f"[tracker] FLIGHT {fid} ended  reason={reason}  "
              f"{self._summary['duration_s']}s  {self._summary['point_count']} pts")
        self._emit("flight_ended", {
            "id": fid, "reason": reason,
            "duration_s": self._summary["duration_s"],
            "point_count": self._summary["point_count"],
            "distance_m": self._summary.get("distance_m"),
            "max_alt_m": self._summary.get("max_alt_m"),
        })

        self.flight_id = None
        self.flight_uuid = None
        self._t0 = None
        self._buf = []

    # ── sampling ──────────────────────────────────────────────────────

    def _sample(self, src):
        s = src.sample()
        now = time.time()
        t_ms = int((now - self._t0) * 1000)

        # NED -> origin-relative, with z flipped to UP-positive for the UI.
        x = _sub(s.get("east"), self._origin[1])
        y = _sub(s.get("north"), self._origin[0])
        dz = _sub(s.get("down"), self._origin[2])
        z = -dz if dz is not None else None

        psrc = s.get("pos_source") or "none"
        self._pos_src_count[psrc] = self._pos_src_count.get(psrc, 0) + 1

        # Mode changes become timeline markers.
        mode = s.get("mode")
        if mode and mode != self._last_mode:
            try:
                self.store.insert_event(self.flight_id, t_ms, "mode", mode)
            except Exception:
                pass
            self._last_mode = mode

        # Running summary.
        sm = self._summary
        sm["point_count"] += 1
        alt = s.get("alt_rf_m") if s.get("alt_rf_m") is not None else z
        if alt is not None and alt > sm["max_alt_m"]:
            sm["max_alt_m"] = alt
        if x is not None and y is not None:
            d = (x * x + y * y) ** 0.5
            if d > sm["max_dist_m"]:
                sm["max_dist_m"] = d
            if self._prev_xyz is not None:
                px, py, pz = self._prev_xyz
                step = ((x - px) ** 2 + (y - py) ** 2 +
                        ((z - pz) ** 2 if (z is not None and pz is not None) else 0)) ** 0.5
                # Ignore implausible jumps (EKF reset / glitch) in path length.
                if step < 5.0:
                    sm["distance_m"] += step
            self._prev_xyz = (x, y, z)
        gs = s.get("groundspeed")
        if gs is not None and gs > sm["max_groundspeed"]:
            sm["max_groundspeed"] = gs
        if s.get("battery_v") is not None:
            sm["battery_end_v"] = s["battery_v"]

        row = (
            self.flight_id, t_ms,
            _r(x, 4), _r(y, 4), _r(z, 4), psrc,
            s.get("alt_rf_m"), s.get("alt_rel_m"),
            s.get("roll"), s.get("pitch"), s.get("yaw"),
            s.get("vx"), s.get("vy"), s.get("vz"), s.get("groundspeed"),
            s.get("rc_roll"), s.get("rc_pitch"), s.get("rc_throttle"), s.get("rc_yaw"),
            s.get("corr_roll"), s.get("corr_pitch"),
            s.get("mode"), s.get("armed"),
            s.get("battery_v"), s.get("battery_pct"), s.get("battery_current"),
            s.get("ekf_ok"), s.get("vibe_x"), s.get("vibe_y"), s.get("vibe_z"),
            s.get("sats"), s.get("gps_fix"),
        )
        self._buf.append(row)

        self._emit_n += 1
        if self._emit_n % EMIT_EVERY_N == 0:
            self._emit("track_point", {
                "flight_id": self.flight_id, "t_ms": t_ms,
                "x": _r(x, 3), "y": _r(y, 3), "z": _r(z, 3), "src": psrc,
                "alt": s.get("alt_rf_m"), "yaw": s.get("yaw"),
                "roll": s.get("roll"), "pitch": s.get("pitch"),
                "gs": s.get("groundspeed"), "mode": mode,
                "batt": s.get("battery_v"),
                "rc": [s.get("rc_roll"), s.get("rc_pitch"),
                       s.get("rc_throttle"), s.get("rc_yaw")],
                "corr": [s.get("corr_roll"), s.get("corr_pitch")],
            })

        if now - self._last_flush >= FLUSH_PERIOD_S:
            self._flush()

    def _flush(self, force=False):
        if not self._buf:
            self._last_flush = time.time()
            return
        rows, self._buf = self._buf, []
        try:
            self.store.insert_points(rows)
        except Exception as e:
            print(f"[tracker] flush failed ({len(rows)} rows lost): "
                  f"{type(e).__name__}: {e}")
        self._last_flush = time.time()

    # ── misc ──────────────────────────────────────────────────────────

    def mark(self, kind, detail=None):
        """Drop a marker on the current flight's timeline. Safe when idle."""
        if self.flight_id is None or self._t0 is None:
            return False
        try:
            self.store.insert_event(self.flight_id,
                                    int((time.time() - self._t0) * 1000),
                                    kind, detail)
            return True
        except Exception:
            return False

    def status(self):
        return {
            "recording": self.flight_id is not None,
            "flight_id": self.flight_id,
            "flight_uuid": self.flight_uuid,
            "elapsed_s": round(time.time() - self._t0, 1) if self._t0 else 0.0,
            "point_count": self._summary.get("point_count", 0),
            "distance_m": _r(self._summary.get("distance_m", 0.0), 2),
            "max_alt_m": _r(self._summary.get("max_alt_m", 0.0), 2),
            "sample_hz": SAMPLE_HZ,
            "source": getattr(self._active_source(), "name", None),
            "video_active": bool(self._video and self._video.active),
            "video_path": self._video.path if self._video else None,
            "attached": self._source is not None,
        }

    def _emit(self, event, payload):
        if self.socketio is None:
            return
        try:
            self.socketio.emit(event, payload)
        except Exception:
            pass


def _r(v, nd):
    return round(v, nd) if isinstance(v, (int, float)) else v
