import math
import threading
import time

from dronekit import VehicleMode
from pymavlink import mavutil


class MovementController:

    def __init__(self, vehicle):
        self._v = vehicle
        self._cancel = threading.Event()
        self._lock = threading.Lock()

    # ── Public interface ──────────────────────────────────────────────

    def takeoff(self, altitude_m: float) -> dict:
        self.clear_cancel()

        res = self._switch_mode("GUIDED_NOGPS")
        if not res["ok"]:
            return res

        hover_thrust = self._get_hover_thrust()
        climb_thrust = hover_thrust + 0.08
        print(f"[MC] takeoff: hover={hover_thrust:.4f} climb={climb_thrust:.4f}")

        res = self._arm()
        if not res["ok"]:
            return res

        start = time.time()
        while True:
            if self._cancel.is_set():
                return self._fail("Takeoff cancelled")
            if time.time() - start > 30:
                self._v.mode = VehicleMode("LAND")
                return self._fail("Takeoff timed out after 30s")

            current_alt = float(self._v.rangefinder.distance or 0)
            self._send_attitude_thrust(climb_thrust)
            print(f"[MC] alt {current_alt:.2f}/{altitude_m:.2f}m")

            if current_alt >= altitude_m * 0.90:
                print("[MC] Target altitude reached, settling…")
                break

            time.sleep(0.1)

        settle_start = time.time()
        while time.time() - settle_start < 2.0:
            self._send_attitude_thrust(hover_thrust)
            time.sleep(0.1)

        try:
            v = self._v.velocity
            spd = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
            if spd > 0.10:
                print(f"[MC] Warning: still moving at settle ({spd:.2f} m/s)")
        except Exception:
            pass

        return {"ok": True, "message": f"Takeoff to {altitude_m}m complete"}

    def move(self, north_m=0.0, east_m=0.0, down_m=0.0, speed=0.3) -> dict:
        self.clear_cancel()

        res = self._switch_mode("GUIDED_NOGPS")
        if not res["ok"]:
            return res

        try:
            local = self._v.location.local_frame
            if local.north is None:
                return self._fail("EKF not ready: local_frame.north is None")
            target_n = local.north + north_m
            target_e = local.east + east_m
            target_d = local.down - down_m  # down axis inverted
        except Exception as e:
            return self._fail(f"Could not read local frame: {e}")

        print(f"[MC] move target NED: ({target_n:.2f}, {target_e:.2f}, {target_d:.2f})")

        start = time.time()
        ok_count = 0

        while True:
            if self._cancel.is_set():
                self._send_stop()
                return self._fail("Move cancelled")

            if time.time() - start > 20:
                self._send_stop()
                return self._fail("Move timed out after 20s")

            self._send_position_target_ned(target_n, target_e, target_d)

            try:
                local = self._v.location.local_frame
                pos_error = math.sqrt(
                    (local.north - target_n) ** 2 +
                    (local.east  - target_e) ** 2
                )
                v = self._v.velocity
                spd = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
            except Exception:
                ok_count = 0
                time.sleep(0.1)
                continue

            if pos_error < 0.15 and spd < 0.10:
                ok_count += 1
            else:
                ok_count = 0

            if ok_count >= 10:
                print("[MC] Move complete (converged)")
                return {"ok": True, "message": "Move complete"}

            time.sleep(0.1)

    def yaw(self, degrees: float, clockwise: bool = True,
            speed_dps: float = 30) -> dict:
        self.clear_cancel()

        res = self._switch_mode("GUIDED_NOGPS")
        if not res["ok"]:
            return res

        direction = 1 if clockwise else -1
        msg = self._v.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            degrees,
            speed_dps,
            direction,
            1,      # relative
            0, 0, 0
        )
        self._v.send_mavlink(msg)

        try:
            start_yaw = math.degrees(self._v.attitude.yaw)
        except Exception:
            start_yaw = 0.0

        timeout = (degrees / speed_dps) + 5
        start = time.time()
        ok_count = 0

        while True:
            if self._cancel.is_set():
                return self._fail("Yaw cancelled")
            if time.time() - start > timeout:
                return self._fail(f"Yaw timed out after {timeout:.1f}s")

            try:
                current_yaw = math.degrees(self._v.attitude.yaw)
                target_yaw  = start_yaw + degrees * direction
                error = abs((target_yaw - current_yaw + 180) % 360 - 180)
            except Exception:
                ok_count = 0
                time.sleep(0.1)
                continue

            if error < 5:
                ok_count += 1
            else:
                ok_count = 0

            if ok_count >= 5:
                print("[MC] Yaw complete")
                return {"ok": True, "message": f"Yaw {degrees}° complete"}

            time.sleep(0.1)

    def hold(self) -> dict:
        self._cancel.set()
        time.sleep(0.05)
        self._send_stop()
        if self._v.mode.name != "GUIDED_NOGPS":
            self._switch_mode("GUIDED_NOGPS")
        return {"ok": True, "message": "Holding position"}

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

        return {"ok": True, "message": "Landed and disarmed"}

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

    def _get_hover_thrust(self) -> float:
        try:
            val = float(self._v.parameters['MOT_THST_HOVER'])
            if 0.1 < val < 0.95:
                return val
        except Exception:
            pass
        print("[MC] MOT_THST_HOVER read failed, using fallback 0.68")
        return 0.68

    def _switch_mode(self, mode_name: str) -> dict:
        try:
            self._v.mode = VehicleMode(mode_name)
            start = time.time()
            while self._v.mode.name != mode_name:
                if time.time() - start > 5:
                    return self._fail(f"Mode switch to {mode_name} timed out")
                time.sleep(0.1)
            return {"ok": True}
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
            return {"ok": True}
        except Exception as e:
            return self._fail(str(e))

    def _send_attitude_thrust(self, thrust: float):
        msg = self._v.message_factory.set_attitude_target_encode(
            0,
            1, 1,
            0b00000111,
            [1, 0, 0, 0],
            0, 0, 0,
            thrust
        )
        self._v.send_mavlink(msg)

    def _send_position_target_ned(self, n: float, e: float, d: float):
        msg = self._v.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # position only
            n, e, d,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        self._v.send_mavlink(msg)

    def _send_stop(self):
        msg = self._v.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,  # velocity only, all zeros
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        self._v.send_mavlink(msg)

    def _fail(self, reason: str) -> dict:
        print(f"[MC] FAIL: {reason}")
        return {"ok": False, "message": reason}
