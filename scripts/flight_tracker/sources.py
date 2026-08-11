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
