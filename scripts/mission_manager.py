import threading
import time

from movement_controller import MovementController


class MissionManager:

    def __init__(self, vehicle):
        self.mc = MovementController(vehicle)
        self._lock = threading.Lock()
        self._status = {
            "active":       False,
            "current_step": None,
            "steps_done":   0,
            "last_result":  None,
            "error":        None,
        }

    def is_busy(self) -> bool:
        return self._status["active"]

    @property
    def status(self) -> dict:
        return dict(self._status)

    # ── Single-command wrappers ───────────────────────────────────────

    def takeoff(self, altitude=1.5) -> dict:
        return self._run_single("takeoff", lambda: self.mc.takeoff(altitude))

    def land(self) -> dict:
        return self._run_single("land", lambda: self.mc.land())

    def hold(self) -> dict:
        return self._run_single("hold", lambda: self.mc.hold())

    def hover(self, duration=3.0) -> dict:
        res = self._run_single("hold", lambda: self.mc.hold())
        if not res["ok"]:
            return res
        time.sleep(duration)
        return {"ok": True, "message": f"Hovered {duration}s"}

    def move_forward(self, distance=1.0, speed=0.3) -> dict:
        return self._run_single("move_forward",
            lambda: self.mc.move(north_m=distance, speed=speed))

    def move_backward(self, distance=1.0, speed=0.3) -> dict:
        return self._run_single("move_backward",
            lambda: self.mc.move(north_m=-distance, speed=speed))

    def move_left(self, distance=1.0, speed=0.3) -> dict:
        return self._run_single("move_left",
            lambda: self.mc.move(east_m=-distance, speed=speed))

    def move_right(self, distance=1.0, speed=0.3) -> dict:
        return self._run_single("move_right",
            lambda: self.mc.move(east_m=distance, speed=speed))

    def move_up(self, distance=0.5, speed=0.3) -> dict:
        return self._run_single("move_up",
            lambda: self.mc.move(down_m=-distance, speed=speed))

    def move_down(self, distance=0.5, speed=0.3) -> dict:
        return self._run_single("move_down",
            lambda: self.mc.move(down_m=distance, speed=speed))

    def yaw_right(self, degrees=90, speed=30) -> dict:
        return self._run_single("yaw_right",
            lambda: self.mc.yaw(degrees, clockwise=True, speed_dps=speed))

    def yaw_left(self, degrees=90, speed=30) -> dict:
        return self._run_single("yaw_left",
            lambda: self.mc.yaw(degrees, clockwise=False, speed_dps=speed))

    def emergency_stop(self):
        self.mc.emergency_stop()

    # ── Mission sequencer ─────────────────────────────────────────────

    def run_mission(self, steps: list, blocking=False) -> dict:
        CMD_MAP = {
            "takeoff":       lambda s: self.takeoff(s.get("altitude", 1.5)),
            "land":          lambda s: self.land(),
            "hold":          lambda s: self.hold(),
            "hover":         lambda s: self.hover(s.get("duration", 3.0)),
            "move_forward":  lambda s: self.move_forward(s.get("distance", 1.0), s.get("speed", 0.3)),
            "move_backward": lambda s: self.move_backward(s.get("distance", 1.0), s.get("speed", 0.3)),
            "move_left":     lambda s: self.move_left(s.get("distance", 1.0), s.get("speed", 0.3)),
            "move_right":    lambda s: self.move_right(s.get("distance", 1.0), s.get("speed", 0.3)),
            "move_up":       lambda s: self.move_up(s.get("distance", 0.5), s.get("speed", 0.3)),
            "move_down":     lambda s: self.move_down(s.get("distance", 0.5), s.get("speed", 0.3)),
            "yaw_right":     lambda s: self.yaw_right(s.get("degrees", 90), s.get("speed", 30)),
            "yaw_left":      lambda s: self.yaw_left(s.get("degrees", 90), s.get("speed", 30)),
        }

        def _execute():
            with self._lock:
                self._status.update({
                    "active":       True,
                    "steps_done":   0,
                    "current_step": None,
                    "last_result":  None,
                    "error":        None,
                })
                self.mc.clear_cancel()

                for i, step in enumerate(steps):
                    cmd = step.get("cmd")
                    if cmd not in CMD_MAP:
                        msg = f"Unknown command: {cmd}"
                        self._status.update({"error": msg, "active": False})
                        return {"ok": False, "steps_done": i, "message": msg}

                    result = CMD_MAP[cmd](step)
                    self._status["steps_done"] = i + 1

                    if not result.get("ok"):
                        if not step.get("ignore_error"):
                            self._status.update({
                                "error":  result.get("message", "Step failed"),
                                "active": False,
                            })
                            return {
                                "ok":        False,
                                "steps_done": i + 1,
                                "message":   self._status["error"],
                            }

                self._status["active"] = False
                return {"ok": True, "steps_done": len(steps), "message": "Mission complete"}

        if blocking:
            return _execute()

        t = threading.Thread(target=_execute, daemon=True)
        t.start()
        return {"ok": True, "steps_done": 0, "message": "Mission started"}

    # ── Internal ──────────────────────────────────────────────────────

    def _run_single(self, step_name: str, fn) -> dict:
        self._status["current_step"] = step_name
        result = fn()
        self._status["last_result"] = result
        return result
