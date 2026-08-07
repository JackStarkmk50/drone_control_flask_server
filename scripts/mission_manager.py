import threading
import time

from movement_controller import MovementController


class MissionManager:

    def __init__(self, vehicle, mc=None):
        # Use the API server's controller if one is handed in.
        #
        # This class used to always build its own MovementController. That
        # instance never had start_rc_override() / start_correction_loop()
        # called on it, so its _rc_sender thread never ran: every set_rc() from
        # a mission wrote into a dict nothing transmitted. Mission takeoff
        # therefore climbed nowhere and timed out into LAND, and mission
        # hold/hover were silent no-ops. It also meant /emergency cancelled a
        # different object than the one the mission was driving, so an
        # emergency stop was followed by the next step re-arming.
        #
        # Falling back to a private controller keeps the old constructor
        # signature working, but the server should always pass its own.
        self.mc = mc if mc is not None else MovementController(vehicle)
        self._owns_mc = mc is None
        self._start_lock = threading.Lock()  # atomic check-and-set for active flag
        self._status = {
            "active":       False,
            "current_step": None,
            "steps_done":   0,
            "last_result":  None,
            "error":        None,
        }

    def is_busy(self) -> bool:
        with self._start_lock:
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
        res = self._run_single("hover", lambda: self.mc.hold())
        if not res["ok"]:
            return res
        # Poll instead of one long sleep so /mission/cancel lands within 100ms
        # rather than after the full hover duration.
        end = time.time() + duration
        while time.time() < end:
            if self.mc._cancel.is_set():
                return {"ok": False, "message": "Hover cancelled"}
            time.sleep(0.1)
        return {"ok": True, "message": f"Hovered {duration}s"}

    # move_forward / backward / left / right / up / down and yaw_right /
    # yaw_left were removed on 2026-08-07 together with MovementController.move()
    # and .yaw(), which were GUIDED-mode primitives this airframe cannot fly.
    #
    # Missions are therefore limited to takeoff / land / hold / hover until
    # RC-based closed-loop move and yaw primitives exist. Directional control is
    # available interactively through /rc and the 'rc' socket event.

    def emergency_stop(self):
        self.mc.emergency_stop()

    # ── Mission sequencer ─────────────────────────────────────────────

    def run_mission(self, steps: list, blocking=False) -> dict:
        with self._start_lock:
            if self._status["active"]:
                return {"ok": False, "steps_done": 0, "message": "Mission already running"}
            self._status["active"] = True  # claim before thread starts — prevents TOCTOU

        CMD_MAP = {
            "takeoff":       lambda s: self.takeoff(s.get("altitude", 1.5)),
            "land":          lambda s: self.land(),
            "hold":          lambda s: self.hold(),
            "hover":         lambda s: self.hover(s.get("duration", 3.0)),
        }

        def _execute():
            # Everything runs inside the try: these two lines used to sit
            # outside it, so an exception here left _status["active"] True
            # forever and every later /mission returned 409 until restart.
            try:
                self._status.update({
                    "steps_done":   0,
                    "current_step": None,
                    "last_result":  None,
                    "error":        None,
                })
                self.mc.clear_cancel()

                for i, step in enumerate(steps):
                    if self.mc._cancel.is_set():
                        msg = "Mission cancelled"
                        self._status["error"] = msg
                        return {"ok": False, "steps_done": i, "message": msg}

                    cmd = step.get("cmd")
                    if cmd not in CMD_MAP:
                        msg = f"Unknown command: {cmd}"
                        self._status.update({"error": msg})
                        return {"ok": False, "steps_done": i, "message": msg}

                    result = CMD_MAP[cmd](step)
                    self._status["steps_done"] = i + 1

                    if not result.get("ok"):
                        if not step.get("ignore_error"):
                            self._status["error"] = result.get("message", "Step failed")
                            return {
                                "ok":         False,
                                "steps_done": i + 1,
                                "message":    self._status["error"],
                            }

                return {"ok": True, "steps_done": len(steps), "message": "Mission complete"}
            finally:
                with self._start_lock:
                    self._status["active"] = False

        if blocking:
            return _execute()

        try:
            t = threading.Thread(target=_execute, daemon=True)
            t.start()
        except Exception as e:
            # active was claimed before the thread existed; release it or the
            # manager is wedged for the life of the process.
            with self._start_lock:
                self._status["active"] = False
            return {"ok": False, "steps_done": 0,
                    "message": f"Could not start mission thread: {e}"}
        return {"ok": True, "steps_done": 0, "message": "Mission started"}

    # ── Internal ──────────────────────────────────────────────────────

    def _run_single(self, step_name: str, fn) -> dict:
        self._status["current_step"] = step_name
        result = fn()
        self._status["last_result"] = result
        return result
