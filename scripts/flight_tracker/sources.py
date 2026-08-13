"""
Telemetry sources for the flight recorder.

The recorder never touches DroneKit directly. It asks a source for a sample dict
and for the armed state. That indirection buys two things:

  * SimSource lets the entire tracker + UI be built and demoed on a laptop with
    no Pixhawk attached, which is the whole point of doing this work while there
    is no space to fly.
  * A future MAVROS / ROS 2 node only has to implement the same two methods.

Every source returns the same flat dict. Missing values are None, never 0.0 —
a real zero and "no data" have to stay distinguishable in the recording.
"""

import math
import time


def _f(val, nd=None):
    """float(val) or None. Never substitutes a fake zero."""
    try:
        if val is None:
            return None
        v = float(val)
        if v != v:          # NaN
            return None
        return round(v, nd) if nd is not None else v
    except (TypeError, ValueError):
        return None


def _i(val):
    try:
        return int(val) if val is not None else None
    except (TypeError, ValueError):
        return None


SAMPLE_KEYS = (
    "north", "east", "down", "pos_source",
    "alt_rf_m", "alt_rel_m",
    "roll", "pitch", "yaw",
    "vx", "vy", "vz", "groundspeed",
    "rc_roll", "rc_pitch", "rc_throttle", "rc_yaw",
    "corr_roll", "corr_pitch",
    "mode", "armed",
    "battery_v", "battery_pct", "battery_current",
    "ekf_ok", "vibe_x", "vibe_y", "vibe_z",
    "sats", "gps_fix",
)


class LiveSource:
    """Samples a DroneKit vehicle plus the MovementController's RC state."""

    name = "live"

    def __init__(self, vehicle, mc=None):
        self._v = vehicle
        self._mc = mc
        # Dead-reckoning accumulator, used only when local_frame is unavailable.
        self._dr = [0.0, 0.0, 0.0]
        self._dr_t = None

    def is_armed(self):
        try:
            return bool(self._v.armed)
        except Exception:
            return False

    def reset(self):
        self._dr = [0.0, 0.0, 0.0]
        self._dr_t = None

    def _position(self, vel, now):
        """
        Returns (north, east, down, source).

        Preferred path is the EKF local frame. If that is unavailable — which is
        what the removed GUIDED move() used to trip on — velocity is integrated
        instead. Dead reckoning drifts, so the source is recorded per point and
        the UI must show it as degraded rather than pretending it is a fix.
        """
        try:
            lf = self._v.location.local_frame
            if lf is not None and lf.north is not None and lf.east is not None:
                self._dr = [lf.north, lf.east, lf.down if lf.down is not None else 0.0]
                self._dr_t = now
                return lf.north, lf.east, lf.down, "ekf"
        except Exception:
            pass

        if vel and vel[0] is not None:
            if self._dr_t is not None:
                dt = now - self._dr_t
                if 0 < dt < 1.0:
                    self._dr[0] += vel[0] * dt
                    self._dr[1] += vel[1] * dt
                    self._dr[2] += vel[2] * dt
            self._dr_t = now
            return self._dr[0], self._dr[1], self._dr[2], "deadreckon"

        return None, None, None, "none"

    def sample(self):
        v = self._v
        now = time.time()
        s = dict.fromkeys(SAMPLE_KEYS)

        try:
            s["mode"] = v.mode.name if v.mode else None
        except Exception:
            pass
        try:
            s["armed"] = 1 if v.armed else 0
        except Exception:
            pass

        vel = [None, None, None]
        try:
            raw = v.velocity
            if raw and len(raw) >= 3:
                vel = [_f(raw[0]), _f(raw[1]), _f(raw[2])]
        except Exception:
            pass
        s["vx"], s["vy"], s["vz"] = vel

        n, e, d, src = self._position(vel, now)
        s["north"], s["east"], s["down"], s["pos_source"] = n, e, d, src

        try:
            rf = _f(v.rangefinder.distance)
            # A rangefinder with no echo reports 0 or a negative; that is not an
            # altitude of zero, it is no reading.
            s["alt_rf_m"] = round(rf, 3) if (rf is not None and rf > 0) else None
        except Exception:
            pass
        try:
            s["alt_rel_m"] = _f(v.location.global_relative_frame.alt, 3)
        except Exception:
            pass

        try:
            s["roll"] = _f(v.attitude.roll, 5)
            s["pitch"] = _f(v.attitude.pitch, 5)
            s["yaw"] = _f(v.attitude.yaw, 5)
        except Exception:
            pass

        try:
            s["groundspeed"] = _f(v.groundspeed, 2)
        except Exception:
            pass

        try:
            s["battery_v"] = _f(v.battery.voltage, 2)
            s["battery_pct"] = _f(v.battery.level, 1)
            s["battery_current"] = _f(v.battery.current, 2)
        except Exception:
            pass

        try:
            s["ekf_ok"] = 1 if v.ekf_ok else 0
        except Exception:
            pass
        try:
            s["vibe_x"] = _f(v.vibration.vibration_x, 3)
            s["vibe_y"] = _f(v.vibration.vibration_y, 3)
            s["vibe_z"] = _f(v.vibration.vibration_z, 3)
        except Exception:
            pass
        try:
            s["sats"] = _i(v.gps_0.satellites_visible)
            s["gps_fix"] = _i(v.gps_0.fix_type)
        except Exception:
            pass

        # Control path. Read straight off the controller's dicts so the record
        # shows what the Pi actually transmitted, not what an endpoint asked for.
        mc = self._mc
        if mc is not None:
            try:
                rc = mc._rc
                s["rc_roll"] = _i(rc.get("roll"))
                s["rc_pitch"] = _i(rc.get("pitch"))
                s["rc_throttle"] = _i(rc.get("throttle"))
                s["rc_yaw"] = _i(rc.get("yaw"))
            except Exception:
                pass
            try:
                s["corr_roll"] = _i(mc._corr.get("roll"))
                s["corr_pitch"] = _i(mc._corr.get("pitch"))
            except Exception:
                pass

        return s


class NavSimSource:
    """
    A simulated aircraft that is *flown by the real controller*.

    SimSource replays a fixed square, which is enough to build a replay UI
    against but proves nothing about navigation. This one instead accepts the
    same RC PWM values WaypointNav sends to the real aircraft, decodes them back
    into a velocity demand exactly the way ArduCopter's LOITER does, and
    integrates. The whole outer loop — position error, speed cap, slew limit,
    world-to-body rotation, PWM scaling — runs unmodified.

    That round trip through PWM is the point. A sign error in the body-frame
    rotation, or a mismatch with DIR_MAP's forward-is-below-neutral convention,
    makes the simulated aircraft accelerate away from the waypoint. That shows
    up in a browser on a laptop rather than in a wall.

    Modelled deliberately, because each part can hide a different bug:

      * LOITER's velocity loop as a first-order lag (VEL_TAU_S). The outer P
        loop has to stay stable on top of a lagging inner loop; an instant
        response would hide overshoot the real aircraft will have.
      * A settable heading. At yaw=0 a pure north command is pure pitch and a
        rotation error is invisible. Fly the same waypoint at yaw=90 degrees and
        it becomes pure roll, so the rotation is actually under test.
      * Stays armed until stop(), unlike SimSource's fixed profile, because
        navigation is open-ended.
    """

    name = "navsim"

    CLIMB_RATE = 0.5     # m/s, used only for the initial climb to hover
    ARM_PAUSE_S = 1.0
    VEL_TAU_S = 0.5      # first-order lag standing in for LOITER's velocity loop

    def __init__(self, alt=1.5, yaw_deg=0.0, battery_v=16.4,
                 pwm_per_ms=100.0, pwm_per_ms_z=200.0):
        self.alt_target = float(alt)
        self.yaw = math.radians(float(yaw_deg))
        self.batt0 = float(battery_v)
        self.pwm_per_ms = float(pwm_per_ms)
        self.pwm_per_ms_z = float(pwm_per_ms_z)

        self._t0 = None
        self._last = None
        self._armed = False
        self._done = False

        # State: position in NED-ish terms plus an up-positive altitude, and the
        # velocity actually achieved (as opposed to demanded).
        self._n = 0.0
        self._e = 0.0
        self._alt = 0.0
        self._vn = 0.0
        self._ve = 0.0
        self._vd = 0.0

        self._rc = {"roll": 1500, "pitch": 1500, "throttle": 1500, "yaw": 1500}

    # ── lifecycle ────────────────────────────────────────────────

    def start(self):
        self._t0 = time.time()
        self._last = self._t0
        self._armed = True
        self._done = False

    def stop(self):
        self._armed = False
        self._done = True

    def is_armed(self):
        return self._armed

    def is_done(self):
        return self._done

    def reset(self):
        pass

    # ── the interface WaypointNav drives ─────────────────────────

    def apply_rc(self, roll, pitch, throttle, yaw):
        """Stands in for MovementController.set_rc()."""
        self._rc = {"roll": int(roll), "pitch": int(pitch),
                    "throttle": int(throttle), "yaw": int(yaw)}

    def pose(self):
        """
        Stands in for the server's pose_fn: origin-relative, x east, y north,
        z up. The simulated aircraft starts at its own origin, so no subtraction
        is needed here.
        """
        self._integrate()
        return {
            "x": self._e, "y": self._n, "z": self._alt,
            "yaw": self.yaw, "src": "ekf", "ekf_ok": True,
            "armed": self._armed, "mode": self._mode(), "t": time.time(),
        }

    def _mode(self):
        if self._t0 is None:
            return "STABILIZE"
        return "LOITER" if (time.time() - self._t0) >= self.ARM_PAUSE_S else "STABILIZE"

    # ── physics ──────────────────────────────────────────────────

    def _integrate(self):
        if self._t0 is None or not self._armed:
            return
        now = time.time()
        dt = now - (self._last or now)
        self._last = now
        if dt <= 0 or dt > 1.0:      # first call, or the thread was descheduled
            return

        t = now - self._t0

        # Climb to the hover altitude before accepting stick input, the same way
        # a real flight reaches altitude before anyone clicks a waypoint.
        if self._alt < self.alt_target - 0.02 and t > self.ARM_PAUSE_S:
            self._alt = min(self.alt_target, self._alt + self.CLIMB_RATE * dt)
            self._vd = -self.CLIMB_RATE
            return

        # Decode PWM back to a body-frame velocity demand. Exact inverse of
        # WaypointNav._control: forward is BELOW neutral on pitch, right is
        # ABOVE neutral on roll.
        d_roll = self._rc["roll"] - 1500
        d_pitch = self._rc["pitch"] - 1500
        d_thr = self._rc["throttle"] - 1500

        right = d_roll / self.pwm_per_ms
        fwd = -d_pitch / self.pwm_per_ms

        # Body -> world. Transpose of the rotation the controller applied.
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        want_n = fwd * cy - right * sy
        want_e = fwd * sy + right * cy
        want_up = d_thr / self.pwm_per_ms_z

        # First-order lag towards the demand.
        k = min(1.0, dt / self.VEL_TAU_S)
        self._vn += (want_n - self._vn) * k
        self._ve += (want_e - self._ve) * k
        vup = -self._vd
        vup += (want_up - vup) * k
        self._vd = -vup

        self._n += self._vn * dt
        self._e += self._ve * dt
        self._alt = max(0.0, self._alt + vup * dt)

    # ── the interface the recorder drives ────────────────────────

    def sample(self):
        s = dict.fromkeys(SAMPLE_KEYS)
        if self._t0 is None:
            return s
        self._integrate()

        t = time.time() - self._t0
        wob = 0.015 * math.sin(t * 2.3)
        span = max(t, 1.0)

        s.update({
            "north": round(self._n, 4),
            "east": round(self._e, 4),
            "down": round(-self._alt, 4),
            "pos_source": "ekf",
            "alt_rf_m": round(max(0.0, self._alt), 3),
            "alt_rel_m": round(max(0.0, self._alt), 3),
            "roll": round(self._ve * 0.08 + wob, 5),
            "pitch": round(-self._vn * 0.08 + wob, 5),
            "yaw": round(self.yaw, 5),
            "vx": round(self._vn, 3), "vy": round(self._ve, 3),
            "vz": round(self._vd, 3),
            "groundspeed": round(math.hypot(self._vn, self._ve), 3),
            "rc_roll": self._rc["roll"], "rc_pitch": self._rc["pitch"],
            "rc_throttle": self._rc["throttle"], "rc_yaw": self._rc["yaw"],
            "corr_roll": int(-wob * 300), "corr_pitch": int(-wob * 300),
            "mode": self._mode(),
            "armed": 1 if self._armed else 0,
            "battery_v": round(self.batt0 - 0.004 * span, 2),
            "battery_pct": round(max(0.0, 100 - 0.25 * span), 1),
            "battery_current": round(18.0 + 4 * abs(self._vd), 2),
            "ekf_ok": 1,
            "vibe_x": round(12 + 3 * abs(self._vn), 2),
            "vibe_y": round(12 + 3 * abs(self._ve), 2),
            "vibe_z": round(18 + 5 * abs(self._vd), 2),
            "sats": 0, "gps_fix": 0,
        })
        return s


class SimSource:
    """
    Synthetic flight, no vehicle required.

    Profile: arm, climb to `alt` on the rangefinder, fly a square of side
    `side` metres at `speed`, return to origin, descend, disarm. Attitude and
    RC channels are derived from the commanded motion so the replay UI, the
    charts and the corrector-vs-attitude comparison all have plausible data to
    render against.
    """

    name = "sim"

    CLIMB_RATE = 0.5      # m/s
    ARM_PAUSE_S = 2.0     # armed-but-still before the climb starts

    def __init__(self, alt=2.0, side=6.0, speed=1.0, hover_s=3.0, battery_v=16.4):
        self.alt = float(alt)
        self.side = float(side)
        self.speed = float(speed)
        self.hover_s = float(hover_s)
        self.batt0 = float(battery_v)
        self._t0 = None
        self._armed = False
        self._done = False

        self._climb_s = self.alt / self.CLIMB_RATE
        self._leg_s = self.side / self.speed
        self._t_climb0 = self.ARM_PAUSE_S
        self._t_climb1 = self._t_climb0 + self._climb_s
        self._t_sq0 = self._t_climb1 + self.hover_s
        self._t_sq1 = self._t_sq0 + 4 * self._leg_s
        self._t_land0 = self._t_sq1 + self.hover_s
        self._t_land1 = self._t_land0 + self._climb_s
        self.total_s = self._t_land1 + 2.0

    def start(self):
        self._t0 = time.time()
        self._armed = True
        self._done = False

    def stop(self):
        self._armed = False
        self._done = True

    def is_armed(self):
        return self._armed

    def is_done(self):
        return self._done

    def reset(self):
        pass

    # legs of the square, as (dnorth, deast) unit vectors
    _LEGS = ((1, 0), (0, 1), (-1, 0), (0, -1))

    def _state(self, t):
        """
        Position and velocity as a closed-form function of elapsed time.

        Computed analytically rather than integrated per sample: integration
        accumulates a little error on every step, so the square visibly failed
        to close back on the origin. A UI built against a reference track that
        does not close would look like a bug in the UI.
        """
        n = e = 0.0
        vn = ve = vd = 0.0
        alt = 0.0
        mode = "LOITER"

        if t < self.ARM_PAUSE_S:
            mode = "GUIDED" if t < 1.0 else "LOITER"
        elif t < self._t_climb1:
            alt = (t - self._t_climb0) * self.CLIMB_RATE
            vd = -self.CLIMB_RATE
        else:
            alt = self.alt
            if t < self._t_sq0:
                pass                                    # hover before the square
            elif t < self._t_sq1:
                ts = t - self._t_sq0
                done = int(ts // self._leg_s)           # completed legs
                frac = (ts - done * self._leg_s) / self._leg_s
                for i in range(min(done, 4)):           # exact corner positions
                    dn, de = self._LEGS[i]
                    n += dn * self.side
                    e += de * self.side
                if done < 4:
                    dn, de = self._LEGS[done]
                    n += dn * self.side * frac
                    e += de * self.side * frac
                    vn, ve = dn * self.speed, de * self.speed
            elif t < self._t_land0:
                pass                                    # hover after the square
            elif t < self._t_land1:
                alt = max(0.0, self.alt - (t - self._t_land0) * self.CLIMB_RATE)
                vd = self.CLIMB_RATE
                mode = "LAND"
            else:
                alt = 0.0
                mode = "LAND"

        return n, e, alt, vn, ve, vd, mode

    def sample(self):
        s = dict.fromkeys(SAMPLE_KEYS)
        if self._t0 is None:
            return s

        t = time.time() - self._t0
        if t >= self.total_s:
            self._armed = False
            self._done = True

        n, e, alt, vn, ve, vd, mode = self._state(t)
        self._pos = [n, e, -alt]
        # A little wobble so charts do not render as dead-flat synthetic lines.
        wob = 0.02 * math.sin(t * 2.3)

        # Nose points along the direction of travel; held on the last heading
        # while hovering. Gives the UI's heading arrow something real to track.
        if vn or ve:
            self._yaw = math.atan2(ve, vn)
        yaw = getattr(self, "_yaw", 0.0)

        s.update({
            "north": round(self._pos[0], 4),
            "east": round(self._pos[1], 4),
            "down": round(self._pos[2], 4),
            "pos_source": "ekf",
            "alt_rf_m": round(max(0.0, alt), 3),
            "alt_rel_m": round(max(0.0, alt), 3),
            # Bank into the direction of travel, as a real aircraft would.
            "roll": round(ve * 0.08 + wob, 5),
            "pitch": round(-vn * 0.08 + wob, 5),
            "yaw": round(yaw + wob * 0.5, 5),
            "vx": round(vn, 3), "vy": round(ve, 3), "vz": round(vd, 3),
            "groundspeed": round(math.hypot(vn, ve), 3),
            "rc_roll": int(1500 + ve * 150),
            "rc_pitch": int(1500 - vn * 150),
            "rc_throttle": int(1500 - vd * 200),
            "rc_yaw": 1500,
            "corr_roll": int(-wob * 300),
            "corr_pitch": int(-wob * 300),
            "mode": mode,
            "armed": 1 if self._armed else 0,
            "battery_v": round(self.batt0 - 0.9 * (t / max(self.total_s, 1)), 2),
            "battery_pct": round(100 - 55 * (t / max(self.total_s, 1)), 1),
            "battery_current": round(18.0 + 4 * abs(vd), 2),
            "ekf_ok": 1,
            "vibe_x": round(12 + 3 * abs(vn), 2),
            "vibe_y": round(12 + 3 * abs(ve), 2),
            "vibe_z": round(18 + 5 * abs(vd), 2),
            "sats": 0,
            "gps_fix": 0,
        })
        return s
