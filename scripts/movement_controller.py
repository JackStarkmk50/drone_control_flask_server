import threading
import time

from dronekit import VehicleMode


# ── RC stream config ─────────────────────────────────────────────
RC_NEUTRAL   = 1500
RC_RATE_HZ   = 20          # Pi → FCU override transmit rate. Raise to 50 for
RC_PERIOD    = 1.0 / RC_RATE_HZ   # crisper response (see notes in BUGS_PLAN_A).

# ── Deadman ──────────────────────────────────────────────────────
# If a non-neutral stick command is held and no refresh arrives within this
# window, every channel is forced back to neutral. Prevents a dropped browser
# / lost network from leaving a movement command latched forever.
RC_DEADMAN_DEFAULT_S = 0.5

# ── Closed-loop attitude correction ──────────────────────────────
CORR_RATE_HZ  = 20
CORR_PERIOD   = 1.0 / CORR_RATE_HZ
CORR_MODE     = "LOITER"   # only correct while in this mode
CORR_SCALE    = 300.0      # PWM per radian of attitude error
CORR_DEADBAND = 0.02       # rad (~1.15°) below which no correction is applied
CORR_CLAMP    = 100        # max |delta| from neutral, PWM

# Sign of each correction axis. -1 means "an error in the positive direction
# produces a BELOW-neutral PWM", which is the levelling (negative feedback)
# direction when the channel is NOT reversed on the flight controller.
#
# Both committed param dumps (drone/mav.parm, drone/scripts/mav.parm) report
# RC1_REVERSED=0 and RC2_REVERSED=0, and DIR_MAP in new_api_server_1.py encodes
# the same non-reversed convention (forward=1350 / backward=1650). Under that
# configuration BOTH axes must be -1.
#
# ⚠ VERIFY BEFORE FLIGHT:  param show RC2_REVERSED
#      returns 0  → leave PITCH_CORR_SIGN = -1  (this is the assumption here)
#      returns 1  → set PITCH_CORR_SIGN = +1 AND flip DIR_MAP's pitch entries
#    Bench check: tilt nose-up, confirm the pitch channel goes BELOW 1500.
PITCH_CORR_SIGN = -1
ROLL_CORR_SIGN  = -1

# ── Takeoff (proportional climb) ─────────────────────────────────
# In LOITER/ALTHOLD the throttle channel is a climb-RATE demand, not a thrust
# level: neutral holds altitude, above neutral climbs at a rate scaled by
# PILOT_SPEED_UP. So the climb command is made proportional to the remaining
# altitude error — large when far away, shrinking to nothing on arrival. The
# aircraft decelerates into the target instead of running at a fixed climb rate
# and being cut dead at 90%, which overshoots and then sags back.
TAKEOFF_TIMEOUT_S      = 30.0
TAKEOFF_LOOP_PERIOD    = 0.1
TAKEOFF_THR_KP         = 120.0   # PWM offset per metre of remaining altitude
TAKEOFF_THR_MAX_OFFSET = 150     # cap, PWM above neutral
TAKEOFF_ARRIVE_M       = 0.10    # stop climbing within this distance of target
TAKEOFF_SETTLE_S       = 2.0     # neutral-throttle settle before declaring done


class MovementController:

    def __init__(self, vehicle):
        self._v = vehicle
        self._cancel = threading.Event()

        # Commanded (teleop / takeoff) stick values. Written by set_rc/rc_hold.
        self._rc = {
            "roll": RC_NEUTRAL, "pitch": RC_NEUTRAL,
            "throttle": RC_NEUTRAL, "yaw": RC_NEUTRAL
        }
        # Closed-loop correction DELTA, kept separate from _rc so the corrector
        # never reads back its own output. Summed in _rc_sender at transmit time.
        self._corr = {"roll": 0, "pitch": 0}

        self._rc_running = False
        self._rc_lock = threading.Lock()

        # Deadman: absolute deadline. 0.0 == disabled (no held command).
        self._deadman_until = 0.0
        self._deadman_tripped = False

        #for closed loop correction
        self._correction_running = False


    def start_correction_loop(self):
        if self._correction_running:
            return
        self._correction_running = True
        threading.Thread(
            target=self._correction_sender,
            daemon=True).start()

    def stop_correction_loop(self):
        self._correction_running = False

    @staticmethod
    def _clamp_delta(v):
        return int(max(-CORR_CLAMP, min(CORR_CLAMP, v)))

    def _correction_sender(self):
        """
        Closed-loop attitude trim.

        Reads actual attitude vs desired (0,0) and produces a small PWM DELTA,
        stored in self._corr. _rc_sender adds that delta to the commanded stick
        value at transmit time, and only on axes the operator is not actively
        driving.

        The delta is written to a dedicated dict rather than back into self._rc.
        Writing into _rc was what made the old version fire exactly once: it
        gated on "_rc pitch and roll are both neutral", then immediately wrote
        non-neutral values into those same keys, so the gate never reopened.
        """
        while self._correction_running:
            active = False
            try:
                if (self._v.armed and
                        self._v.mode.name == CORR_MODE):

                    att = self._v.attitude
                    # pitch: + = nose up
                    # roll:  + = right tilt
                    pitch_err = att.pitch  # radians
                    roll_err  = att.roll

                    if (abs(pitch_err) > CORR_DEADBAND or
                            abs(roll_err) > CORR_DEADBAND):
                        p_corr = self._clamp_delta(
                            pitch_err * PITCH_CORR_SIGN * CORR_SCALE)
                        r_corr = self._clamp_delta(
                            roll_err * ROLL_CORR_SIGN * CORR_SCALE)
                        with self._rc_lock:
                            self._corr["pitch"] = p_corr
                            self._corr["roll"]  = r_corr
                        active = True

            except Exception as e:
                print(f"[MC] Correction error: {e}")

            # Disarmed, wrong mode, error, or inside the deadband: drop the trim
            # rather than leaving a stale value latched into the RC stream.
            if not active:
                with self._rc_lock:
                    self._corr["pitch"] = 0
                    self._corr["roll"]  = 0

            time.sleep(CORR_PERIOD)


        # Add method: start RC thread
    def start_rc_override(self):
        if self._rc_running:
            return
        self._rc_running = True
        threading.Thread(
            target=self._rc_sender,
            daemon=True).start()

    def _check_deadman_locked(self):
        """Caller must hold _rc_lock. Centres everything if the hold expired."""
        if self._deadman_until <= 0.0:
            return
        if all(v == RC_NEUTRAL for v in self._rc.values()):
            self._deadman_until = 0.0     # nothing held, nothing to guard
            return
        if time.time() > self._deadman_until:
            for ch in self._rc:
                self._rc[ch] = RC_NEUTRAL
            self._deadman_until = 0.0
            if not self._deadman_tripped:
                self._deadman_tripped = True
                print("[MC] DEADMAN tripped — no refresh from client, "
                      "all channels forced to neutral")

    # Background sender
    def _rc_sender(self):
        while self._rc_running:
            try:
                with self._rc_lock:
                    self._check_deadman_locked()
                    r = self._rc["roll"]
                    p = self._rc["pitch"]
                    t = self._rc["throttle"]
                    y = self._rc["yaw"]
                    # Correction trims only the axes not being driven.
                    if r == RC_NEUTRAL:
                        r = RC_NEUTRAL + self._corr["roll"]
                    if p == RC_NEUTRAL:
                        p = RC_NEUTRAL + self._corr["pitch"]
                msg = self._v.message_factory\
                    .rc_channels_override_encode(
                    1,1, r,p,t,y, 0,0,0,0)
                self._v.send_mavlink(msg)
            except Exception as e:
                # Must never propagate: an uncaught exception here kills the
                # thread while _rc_running stays True, which makes
                # start_rc_override() a no-op and leaves RC permanently dead.
                print(f"[MC] RC send error: {e}")
            time.sleep(RC_PERIOD)

    def touch_deadman(self, timeout_s=RC_DEADMAN_DEFAULT_S):
        """Refresh the hold deadline. timeout_s <= 0 disables the deadman."""
        with self._rc_lock:
            if timeout_s and timeout_s > 0:
                self._deadman_until = time.time() + timeout_s
                self._deadman_tripped = False
            else:
                self._deadman_until = 0.0

    # Set RC values
    def set_rc(self, roll=1500, pitch=1500,
            throttle=1500, yaw=1500):
        def c(v): return max(1000,min(2000,int(v)))
        with self._rc_lock:
            self._rc = {
                "roll":     c(roll),
                "pitch":    c(pitch),
                "throttle": c(throttle),
                "yaw":      c(yaw)
            }

    # Reset channel to center
    def rc_hold(self, *channels):
        with self._rc_lock:
            for ch in (channels or
                ["roll","pitch","throttle","yaw"]):
                self._rc[ch] = RC_NEUTRAL
            if all(v == RC_NEUTRAL for v in self._rc.values()):
                self._deadman_until = 0.0

    # Stop RC thread
    def stop_rc_override(self):
        self._rc_running = False

    # ── Public interface ──────────────────────────────────────────────

    def takeoff(self, altitude_m: float) -> dict:
        self.clear_cancel()

        res = self._switch_mode(CORR_MODE)
        if not res["ok"]:
            return res

        res = self._arm()
        if not res["ok"]:
            return res

        print(f"[MC] Taking off to {altitude_m}m")

        start = time.time()
        while True:
            if self._cancel.is_set():
                self.rc_hold()
                return self._fail("Takeoff cancelled")

            if time.time() - start > TAKEOFF_TIMEOUT_S:
                self.rc_hold()
                self._v.mode = VehicleMode("LAND")
                return self._fail(
                    f"Takeoff timed out after {TAKEOFF_TIMEOUT_S:.0f}s")

            rf_dist = self._v.rangefinder.distance
            if rf_dist is None:
                self.rc_hold()
                self._v.mode = VehicleMode("LAND")
                return self._fail(
                    "Rangefinder not ready (None) — aborting takeoff")

            current_alt = float(rf_dist)
            err = altitude_m - current_alt

            if err <= TAKEOFF_ARRIVE_M:
                break

            # Proportional climb-rate demand: full offset while far from the
            # target, tapering to zero on arrival.
            offset = int(max(0, min(TAKEOFF_THR_MAX_OFFSET,
                                    TAKEOFF_THR_KP * err)))
            self.set_rc(throttle=RC_NEUTRAL + offset)
            print(f"[MC] alt {current_alt:.2f}/{altitude_m:.2f}m  thr+{offset}")
            time.sleep(TAKEOFF_LOOP_PERIOD)

        # Neutral throttle and let the altitude controller absorb the residual
        # climb rate before reporting success.
        print(f"[MC] Target reached, settling {TAKEOFF_SETTLE_S:.0f}s…")
        self.rc_hold()
        settle_start = time.time()
        while time.time() - settle_start < TAKEOFF_SETTLE_S:
            if self._cancel.is_set():
                return self._fail("Takeoff cancelled during settle")
            time.sleep(0.1)

        try:
            final_alt = float(self._v.rangefinder.distance)
            settled = f" (settled at {final_alt:.2f}m)"
        except Exception:
            settled = ""
        print(f"[MC] Takeoff complete{settled}")
        return {"ok": True,
                "message": f"Takeoff to {altitude_m}m complete{settled}"}

    # NOTE: move() and yaw() were removed on 2026-08-07.
    #
    # Both were GUIDED-mode implementations left over from the abandoned
    # migration (see new_changes/to_guided/). move() streamed
    # SET_POSITION_TARGET_LOCAL_NED while switching the vehicle to LOITER —
    # ArduCopter only honours those setpoints in GUIDED, so the packets were
    # discarded and the call always ran to its 20s timeout before returning.
    # It was also gated behind an "EKF not ready" check that never passes
    # indoors without GPS. yaw() used MAV_CMD_CONDITION_YAW and forced GUIDED,
    # which fought the 20Hz RC override stream.
    #
    # Directional control is RC-only now: set_rc() / rc_hold() driven by
    # DIR_MAP in new_api_server_1.py, including yaw_left / yaw_right.
    # Closed-loop RC-based move/yaw primitives are the next step.

    def hold(self) -> dict:
        """
        Stop all movement: centre every channel, stay in the current mode.

        Previously this sent a BODY_NED zero-velocity setpoint and forced
        GUIDED. The setpoint was ignored outside GUIDED, and the mode switch
        pulled the aircraft out of the mode the RC stream targets.
        """
        self._cancel.set()
        self.rc_hold()
        return self._success("Holding — all channels centred")

    def land(self) -> dict:
        self.cancel()
        try:
            self._v.mode = VehicleMode("LAND")
        except Exception as e:
            return self._fail(f"Could not switch to LAND: {e}")

        start = time.time()
        while self._v.armed:
            if time.time() - start > 30:
                return self._fail("Land timed out: still armed after 30s")
            time.sleep(0.5)

        return self._success("Landed and disarmed")

    def emergency_stop(self):
        self.cancel()
        try:
            self._v.mode = VehicleMode("LAND")
        except Exception:
            pass
        time.sleep(1)
        try:
            self._v.armed = False
        except Exception:
            pass

    def cancel(self):
        self._cancel.set()

    def clear_cancel(self):
        self._cancel.clear()

    # ── Private helpers ───────────────────────────────────────────────

    def _switch_mode(self, mode_name: str) -> dict:
        try:
            self._v.mode = VehicleMode(mode_name)
            start = time.time()
            while self._v.mode.name != mode_name:
                if time.time() - start > 5:
                    return self._fail(f"Mode switch to {mode_name} timed out")
                time.sleep(0.1)
            return self._success(f"Mode switched to {mode_name}")
        except Exception as e:
            return self._fail(str(e))

    def _arm(self) -> dict:
        try:
            self._v.armed = True
            start = time.time()
            while not self._v.armed:
                if time.time() - start > 15:
                    return self._fail("Arm timed out after 15s")
                print("[MC] Waiting for arm…")
                time.sleep(0.5)
            print("[MC] Armed!")
            return self._success("Armed")
        except Exception as e:
            return self._fail(str(e))

    def _fail(self, reason: str) -> dict:
        print(f"[MC] FAIL: {reason}")
        return {"ok": False, "message": reason}
    
    def _success(self, reason: str) -> dict:
        print(f"[MC] Success: {reason}")
        return {"ok": True, "message": reason}
