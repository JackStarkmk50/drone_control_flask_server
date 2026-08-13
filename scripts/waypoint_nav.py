"""
Click-to-waypoint navigation for a GPS-denied indoor quadcopter.

The operator clicks a point on the track plot in the browser; the aircraft flies
there and holds. This module is the closed loop that makes that happen.


WHY NOT GUIDED
--------------
The obvious implementation is GUIDED + SET_POSITION_TARGET_LOCAL_NED, letting
the flight controller's own position controller do the work. That is still the
better answer *if* the EKF has a real horizontal position source, and it is
available here as mode="guided".

It is not the default, because:

  * GUIDED was tried on this airframe before and abandoned (see the removal note
    in movement_controller.py). ArduCopter only honours position setpoints in
    GUIDED, and the previous attempt streamed them while in LOITER, so every
    packet was discarded.
  * The RC override stream at 20 Hz is the actuation path that has actually
    flown. Switching modes mid-flight to a path with no flight time behind it,
    on an aircraft with a questionable position estimate, is the riskier of the
    two options.

So the default is mode="rc": a slow outer position loop on the Pi that produces
stick commands, cascaded on top of LOITER's own velocity loop in the FCU.


THE CONTROL LAW
---------------
In LOITER, the roll/pitch sticks are a *velocity* demand, not a lean angle, and
the throttle stick is a *climb-rate* demand. Full deflection (500 PWM from
neutral) corresponds to LOIT_SPEED horizontally and PILOT_SPEED_UP vertically.
Both are read off the vehicle at attach time.

That makes the outer loop straightforward:

    position error (world) -> desired velocity (world, P + speed cap)
                           -> desired velocity (body, rotated by yaw)
                           -> stick PWM (scaled by LOIT_SPEED)

and when the sticks return to neutral, LOITER holds the position it arrived at.
No mode change, no fighting the corrector, and the existing deadman still backs
everything up.

Cascading a slow P loop (NAV_KP ~0.8/s) on top of LOITER's much faster velocity
loop is stable by separation of timescales. The commanded velocity is also
slew-limited, so accepting a waypoint does not apply a step input.


FRAME
-----
One frame throughout, identical to the flight tracker and the browser plot:

    x = metres EAST of the arm point
    y = metres NORTH of the arm point
    z = metres UP from the arm point

The arm-point origin comes from the tracker so a waypoint clicked on the plot
lands in exactly the frame the plot was drawn in. Getting this wrong is the
easiest way to fly into a wall, so there is one definition and everything reads
it from here.


ALTITUDE
--------
By default a waypoint carries no altitude and the throttle stick stays neutral,
which leaves altitude entirely to the FCU's own altitude hold. The aircraft
flies to the point at whatever height it is already at.

That is deliberate for v1. Horizontal position from the EKF and vertical
position from a rangefinder have very different trust levels indoors, and there
is no reason to put both in the loop at once. Pass z explicitly to opt in.


SAFETY
------
The loop refuses to run, and aborts if already running, on any of:

    * not armed
    * mode is not LOITER
    * pos_source is not "ekf"  (dead reckoning must never drive a position loop)
    * ekf_ok false
    * position sample older than POS_STALE_S
    * outside MAX_RADIUS_M of the arm point
    * leg timeout
    * any manual stick input, /hold, /land, /rtl, /emergency, or disarm

Abort always centres the sticks and leaves the mode alone: LOITER then holds.
"""

import math
import threading
import time

from flask import Blueprint, jsonify, request


RC_NEUTRAL = 1500

# ── Outer loop ───────────────────────────────────────────────────
NAV_RATE_HZ   = 10.0
NAV_PERIOD    = 1.0 / NAV_RATE_HZ

NAV_KP        = 0.8      # 1/s.  desired speed = KP * distance, then capped
NAV_KP_Z      = 0.7      # 1/s.  vertical equivalent, only used when z is given
NAV_SPEED_MAX = 0.8      # m/s horizontal ceiling regardless of how far away
NAV_VZ_MAX    = 0.4      # m/s vertical ceiling
NAV_ACCEL_MAX = 0.5      # m/s^2 slew limit on the commanded velocity vector

# Hard PWM clamp, applied after every other limit. Belt and braces: even with a
# mis-read LOIT_SPEED this caps how hard the aircraft can be commanded.
NAV_PWM_MAX   = 150

# ── Arrival ──────────────────────────────────────────────────────
ARRIVE_RADIUS_M = 0.35
ARRIVE_ALT_M    = 0.25
ARRIVE_HOLD_S   = 1.0    # must stay inside the radius this long to count

# ── Limits and timeouts ──────────────────────────────────────────
MAX_RADIUS_M   = 15.0    # from the arm point; waypoints outside are rejected
MIN_ALT_M      = 0.3
MAX_ALT_M      = 4.0
POS_STALE_S    = 1.0
LEG_TIMEOUT_PAD_S = 10.0  # timeout = distance / NAV_SPEED_MAX * 3 + this

# Deadman refresh. MovementController centres every channel if a non-neutral
# command is not refreshed inside its window, so the nav loop has to keep
# touching it. Three nav periods of slack: one missed cycle must not trip it,
# but a dead nav thread must.
NAV_DEADMAN_S = NAV_PERIOD * 3.0

# ── Fallbacks when the params cannot be read ─────────────────────
# Both match mav.parm. If a read fails the loop still runs, just with the
# documented defaults rather than a guess.
LOIT_SPEED_CMS_DEFAULT     = 500.0    # 5.0 m/s at full stick
PILOT_SPEED_UP_CMS_DEFAULT = 250.0    # 2.5 m/s at full stick
FULL_STICK_PWM = 500.0


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class Waypoint:
    __slots__ = ("x", "y", "z", "seq", "added_at")

    def __init__(self, x, y, z=None, seq=0):
        self.x = float(x)
        self.y = float(y)
        self.z = None if z is None else float(z)
        self.seq = seq
        self.added_at = time.time()

    def to_dict(self):
        return {"seq": self.seq, "x": round(self.x, 3), "y": round(self.y, 3),
                "z": None if self.z is None else round(self.z, 3)}


class WaypointNav:
    """
    Owns the waypoint queue and the outer position loop.

    Deliberately holds no reference to the vehicle or the MovementController
    directly — everything goes through the two callables passed to attach(), so
    the identical control law can be driven against the simulator in a browser
    with no Pixhawk present. The PWM round-trip is part of that: the simulator
    decodes stick values back into velocity, which means a sign error in the
    body-frame rotation shows up on a laptop instead of in a wall.
    """

    def __init__(self, socketio=None, mode="rc"):
        self._sio = socketio
        self.mode = mode

        self._pose_fn = None      # () -> dict, see _pose() for the contract
        self._send_fn = None      # (roll, pitch, throttle, yaw) -> None
        self._mark_fn = None      # (kind, detail) -> None, timeline markers
        self._param_fn = None     # (name) -> float|None

        self._q = []
        self._seq = 0
        self._current = None
        self._lock = threading.Lock()

        self._running = False
        self._thread = None

        self._state = "idle"      # idle | flying | holding | aborted
        self._reason = None
        self._leg_start = 0.0
        self._inside_since = None
        self._v_cmd = [0.0, 0.0]  # last commanded world velocity, for the slew limit
        self._last_err = None
        self._reached = 0

        self.pwm_per_ms   = FULL_STICK_PWM / (LOIT_SPEED_CMS_DEFAULT / 100.0)
        self.pwm_per_ms_z = FULL_STICK_PWM / (PILOT_SPEED_UP_CMS_DEFAULT / 100.0)

    # ── Wiring ───────────────────────────────────────────────────

    def attach(self, pose_fn, send_fn, mark_fn=None, param_fn=None):
        self._pose_fn = pose_fn
        self._send_fn = send_fn
        self._mark_fn = mark_fn
        self._param_fn = param_fn
        self._read_params()

    def _read_params(self):
        """
        Scale factors come from the aircraft, not from a constant, because
        LOIT_SPEED is exactly the sort of parameter that gets retuned and then
        silently invalidates a hard-coded PWM-per-m/s.
        """
        if self._param_fn is None:
            return
        try:
            loit = self._param_fn("LOIT_SPEED")
            if loit and loit > 0:
                self.pwm_per_ms = FULL_STICK_PWM / (float(loit) / 100.0)
        except Exception:
            pass
        try:
            up = self._param_fn("PILOT_SPEED_UP")
            if up and up > 0:
                self.pwm_per_ms_z = FULL_STICK_PWM / (float(up) / 100.0)
        except Exception:
            pass
        print(f"[nav] scale: {self.pwm_per_ms:.1f} PWM per m/s horizontal, "
              f"{self.pwm_per_ms_z:.1f} vertical")

    def start(self):
        with self._lock:
            if self._running:
                return
            self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[nav] loop started @{NAV_RATE_HZ:.0f}Hz  mode={self.mode}")

    def stop(self):
        self._running = False

    # ── Queue API ────────────────────────────────────────────────

    def goto(self, x, y, z=None, replace=True):
        """Fly to one point. replace=True clears anything already queued."""
        ok, msg = self._validate(x, y, z)
        if not ok:
            return {"ok": False, "message": msg}

        with self._lock:
            if replace:
                self._q = []
                self._current = None
                self._reset_leg()
            self._seq += 1
            wp = Waypoint(x, y, z, self._seq)
            self._q.append(wp)
            self._state = "flying"
            self._reason = None

        self._mark("waypoint", f"goto ({wp.x:.2f}E, {wp.y:.2f}N)"
                               + ("" if wp.z is None else f" @{wp.z:.2f}m"))
        self._emit_status()
        return {"ok": True, "message": f"Waypoint {wp.seq} accepted",
                "waypoint": wp.to_dict()}

    def queue(self, points):
        """Append several points. Rejects the whole batch if any point is bad."""
        checked = []
        for i, p in enumerate(points):
            try:
                x, y = p["x"], p["y"]
            except (TypeError, KeyError):
                return {"ok": False, "message": f"Point {i} missing x/y"}
            z = p.get("z")
            ok, msg = self._validate(x, y, z)
            if not ok:
                return {"ok": False, "message": f"Point {i}: {msg}"}
            checked.append((x, y, z))

        with self._lock:
            added = []
            for x, y, z in checked:
                self._seq += 1
                wp = Waypoint(x, y, z, self._seq)
                self._q.append(wp)
                added.append(wp.to_dict())
            self._state = "flying"
            self._reason = None

        self._mark("waypoint", f"queued {len(added)} points")
        self._emit_status()
        return {"ok": True, "message": f"{len(added)} waypoints queued",
                "waypoints": added}

    def clear(self):
        with self._lock:
            n = len(self._q) + (1 if self._current else 0)
            self._q = []
            self._current = None
            self._reset_leg()
            self._state = "idle"
        self._centre()
        self._emit_status()
        return {"ok": True, "message": f"Cleared {n} waypoints"}

    def abort(self, reason="aborted"):
        """
        Stop navigating and centre the sticks. Mode is left alone on purpose —
        LOITER then holds the position it is at, which is the safe outcome. Safe
        to call when nothing is running.
        """
        with self._lock:
            if not self._q and self._current is None and self._state in ("idle", "aborted"):
                return {"ok": True, "message": "Nothing to abort"}
            self._q = []
            self._current = None
            self._reset_leg()
            self._state = "aborted"
            self._reason = reason
        self._centre()
        self._mark("waypoint", f"abort: {reason}")
        self._emit("nav_aborted", {"reason": reason})
        self._emit_status()
        print(f"[nav] ABORT — {reason}")
        return {"ok": True, "message": f"Navigation aborted: {reason}"}

    # ── Validation ───────────────────────────────────────────────

    def _validate(self, x, y, z):
        try:
            x = float(x)
            y = float(y)
        except (TypeError, ValueError):
            return False, "x and y must be numbers"
        if x != x or y != y:
            return False, "x and y must be numbers"

        r = math.hypot(x, y)
        if r > MAX_RADIUS_M:
            return False, (f"{r:.1f}m from the arm point exceeds the "
                           f"{MAX_RADIUS_M:.0f}m limit")
        if z is not None:
            try:
                z = float(z)
            except (TypeError, ValueError):
                return False, "z must be a number or null"
            if not (MIN_ALT_M <= z <= MAX_ALT_M):
                return False, (f"altitude {z:.2f}m outside "
                               f"{MIN_ALT_M}–{MAX_ALT_M}m")

        p = self._pose()
        if p is None:
            return False, "No position — nav needs an armed aircraft with a fix"
        blocked = self._gate(p)
        if blocked:
            return False, blocked
        return True, None

    def _gate(self, p):
        """Returns a reason string when navigation must not run, else None."""
        if not p.get("armed"):
            return "not armed"
        if (p.get("mode") or "").upper() != "LOITER":
            return f"mode is {p.get('mode')}, needs LOITER"
        if p.get("src") != "ekf":
            return (f"position source is '{p.get('src')}' — dead reckoning "
                    f"drifts and must not drive a position loop")
        if not p.get("ekf_ok"):
            return "EKF not healthy"
        if p.get("t") is not None and (time.time() - p["t"]) > POS_STALE_S:
            return "position estimate is stale"
        if p.get("x") is None or p.get("y") is None:
            return "no position estimate"
        if math.hypot(p["x"], p["y"]) > MAX_RADIUS_M:
            return f"aircraft is beyond the {MAX_RADIUS_M:.0f}m safety radius"
        return None

    # ── Loop ─────────────────────────────────────────────────────

    def _pose(self):
        if self._pose_fn is None:
            return None
        try:
            return self._pose_fn()
        except Exception as e:
            print(f"[nav] pose read failed: {type(e).__name__}: {e}")
            return None

    def _reset_leg(self):
        self._leg_start = 0.0
        self._inside_since = None
        self._v_cmd = [0.0, 0.0]
        self._last_err = None

    def _centre(self):
        if self._send_fn is None:
            return
        try:
            self._send_fn(RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL)
        except Exception as e:
            print(f"[nav] centre failed: {type(e).__name__}: {e}")

    def _loop(self):
        last = time.time()
        while self._running:
            now = time.time()
            dt = now - last
            last = now
            try:
                self._step(dt)
            except Exception as e:
                # An uncaught exception here would kill the thread while the
                # aircraft is holding a non-neutral stick command. Centre first,
                # then abort, then keep the loop alive.
                print(f"[nav] step error: {type(e).__name__}: {e}")
                self._centre()
                try:
                    self.abort(f"internal error: {type(e).__name__}")
                except Exception:
                    pass
            time.sleep(NAV_PERIOD)

    def _step(self, dt):
        with self._lock:
            if self._current is None:
                if not self._q:
                    return
                self._current = self._q.pop(0)
                self._reset_leg()
                self._leg_start = time.time()
                print(f"[nav] leg -> waypoint {self._current.seq} "
                      f"({self._current.x:.2f}E, {self._current.y:.2f}N)")
            wp = self._current

        p = self._pose()
        if p is None:
            self.abort("lost telemetry")
            return

        blocked = self._gate(p)
        if blocked:
            self.abort(blocked)
            return

        ex = wp.x - p["x"]
        ey = wp.y - p["y"]
        dist = math.hypot(ex, ey)
        ez = None if wp.z is None else (wp.z - (p.get("z") or 0.0))
        self._last_err = (ex, ey, ez, dist)

        # Arrival: inside the radius (and altitude band, when one was asked for)
        # continuously for ARRIVE_HOLD_S. A momentary pass-through at speed is
        # not an arrival.
        at_alt = ez is None or abs(ez) <= ARRIVE_ALT_M
        if dist <= ARRIVE_RADIUS_M and at_alt:
            if self._inside_since is None:
                self._inside_since = time.time()
            elif time.time() - self._inside_since >= ARRIVE_HOLD_S:
                self._arrive(wp)
                return
        else:
            self._inside_since = None

        timeout = dist / NAV_SPEED_MAX * 3.0 + LEG_TIMEOUT_PAD_S
        if time.time() - self._leg_start > timeout:
            self.abort(f"waypoint {wp.seq} timed out after {timeout:.0f}s "
                       f"({dist:.2f}m short)")
            return

        roll, pitch, throttle = self._control(p, ex, ey, ez, dist, dt)
        try:
            self._send_fn(roll, pitch, throttle, RC_NEUTRAL)
        except Exception as e:
            print(f"[nav] send failed: {type(e).__name__}: {e}")
            self.abort("could not transmit")

    def _control(self, p, ex, ey, ez, dist, dt):
        """
        World position error -> body-frame stick PWM.

        Returns (roll, pitch, throttle) as absolute PWM values.
        """
        # 1. Desired world velocity: proportional, capped. The cap is what makes
        #    a distant waypoint a constant-speed cruise rather than a lunge.
        if dist > 1e-6:
            speed = min(NAV_SPEED_MAX, NAV_KP * dist)
            want_e = speed * ex / dist          # east  component, m/s
            want_n = speed * ey / dist          # north component, m/s
        else:
            want_e = want_n = 0.0

        # 2. Slew limit, so accepting a waypoint is a ramp rather than a step.
        if dt > 0:
            max_d = NAV_ACCEL_MAX * min(dt, 0.5)
            dv_e = _clamp(want_e - self._v_cmd[0], -max_d, max_d)
            dv_n = _clamp(want_n - self._v_cmd[1], -max_d, max_d)
            self._v_cmd[0] += dv_e
            self._v_cmd[1] += dv_n
        v_e, v_n = self._v_cmd

        # 3. World -> body. ArduPilot yaw is clockwise from north, so the nose
        #    unit vector is (north=cos y, east=sin y) and the right-wing vector
        #    is that rotated 90 degrees clockwise, (north=-sin y, east=cos y).
        yaw = p.get("yaw") or 0.0
        cy, sy = math.cos(yaw), math.sin(yaw)
        fwd   =  v_n * cy + v_e * sy
        right = -v_n * sy + v_e * cy

        # 4. Stick PWM. DIR_MAP in the server defines forward as BELOW neutral
        #    on pitch and right as ABOVE neutral on roll; match it exactly, or
        #    the aircraft flies away from the waypoint at increasing speed.
        d_pitch = _clamp(-fwd   * self.pwm_per_ms, -NAV_PWM_MAX, NAV_PWM_MAX)
        d_roll  = _clamp( right * self.pwm_per_ms, -NAV_PWM_MAX, NAV_PWM_MAX)

        # 5. Altitude, only when the waypoint asked for one. Otherwise neutral
        #    throttle and the FCU's altitude hold keeps the current height.
        if ez is None:
            d_thr = 0.0
        else:
            vz = _clamp(NAV_KP_Z * ez, -NAV_VZ_MAX, NAV_VZ_MAX)
            d_thr = _clamp(vz * self.pwm_per_ms_z, -NAV_PWM_MAX, NAV_PWM_MAX)

        return (int(round(RC_NEUTRAL + d_roll)),
                int(round(RC_NEUTRAL + d_pitch)),
                int(round(RC_NEUTRAL + d_thr)))

    def _arrive(self, wp):
        with self._lock:
            self._current = None
            self._reset_leg()
            self._reached += 1
            more = bool(self._q)
            self._state = "flying" if more else "holding"
        self._centre()
        print(f"[nav] reached waypoint {wp.seq}"
              + ("" if more else " — holding"))
        self._mark("waypoint", f"reached {wp.seq}")
        self._emit("nav_reached", {"seq": wp.seq, "waypoint": wp.to_dict(),
                                   "remaining": len(self._q)})
        self._emit_status()

    # ── Status / events ──────────────────────────────────────────

    def status(self):
        with self._lock:
            cur = self._current.to_dict() if self._current else None
            q = [w.to_dict() for w in self._q]
            state = self._state
            reason = self._reason
            reached = self._reached
        err = self._last_err
        p = self._pose()
        return {
            "available": True,
            "running": self._running,
            "mode": self.mode,
            "state": state,
            "reason": reason,
            "current": cur,
            "queue": q,
            "queued": len(q),
            "reached": reached,
            "distance_m": None if not err else round(err[3], 3),
            "ready": self._gate(p) is None if p else False,
            "blocked_by": self._gate(p) if p else "no telemetry",
            "position": None if not p else {
                "x": None if p.get("x") is None else round(p["x"], 3),
                "y": None if p.get("y") is None else round(p["y"], 3),
                "z": None if p.get("z") is None else round(p["z"], 3),
                "yaw": None if p.get("yaw") is None else round(p["yaw"], 4),
                "src": p.get("src"), "mode": p.get("mode"),
                "armed": bool(p.get("armed")),
            },
            "limits": {"max_radius_m": MAX_RADIUS_M, "min_alt_m": MIN_ALT_M,
                       "max_alt_m": MAX_ALT_M, "speed_max": NAV_SPEED_MAX,
                       "arrive_radius_m": ARRIVE_RADIUS_M},
        }

    def _mark(self, kind, detail):
        if self._mark_fn is None:
            return
        try:
            self._mark_fn(kind, detail)
        except Exception:
            pass

    def _emit(self, event, payload):
        if self._sio is None:
            return
        try:
            self._sio.emit(event, payload)
        except Exception:
            pass

    def _emit_status(self):
        self._emit("nav_status", self.status())


# ─── Blueprint ───────────────────────────────────────────────────

def make_nav_bp(nav):
    bp = Blueprint("nav", __name__)

    def _body():
        return request.get_json(silent=True) or {}

    @bp.route("/goto", methods=["POST"])
    def nav_goto():
        d = _body()
        if "x" not in d or "y" not in d:
            return jsonify({"success": False,
                            "message": "x and y required (metres east/north "
                                       "of the arm point)"}), 400
        r = nav.goto(d["x"], d["y"], d.get("z"),
                     replace=bool(d.get("replace", True)))
        return jsonify({"success": r["ok"], **r}), (200 if r["ok"] else 400)

    @bp.route("/queue", methods=["POST"])
    def nav_queue():
        d = _body()
        pts = d.get("points")
        if not isinstance(pts, list) or not pts:
            return jsonify({"success": False,
                            "message": "points must be a non-empty list"}), 400
        r = nav.queue(pts)
        return jsonify({"success": r["ok"], **r}), (200 if r["ok"] else 400)

    @bp.route("/status", methods=["GET"])
    def nav_status():
        return jsonify({"success": True, **nav.status()})

    @bp.route("/abort", methods=["POST"])
    def nav_abort():
        r = nav.abort(_body().get("reason", "operator"))
        return jsonify({"success": True, **r})

    @bp.route("/clear", methods=["POST"])
    def nav_clear():
        return jsonify({"success": True, **nav.clear()})

    return bp
