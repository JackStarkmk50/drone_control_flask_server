"""
Single-reader frame hub.

Problem this solves: cv2.VideoCapture.read() CONSUMES a frame. If the MJPEG
stream generator and the flight video recorder both call read() on the same
capture, they split the stream — each gets roughly half the frames, and the
live view visibly stutters the moment recording starts. Two MJPEG viewers
already have this problem today.

Fix: exactly one thread reads the capture. Everyone else takes the latest frame
out of a slot. Consumers that fall behind skip frames instead of blocking the
grabber, which is the correct behaviour for both a live view and a fixed-rate
recording.
"""

import threading
import time


class FrameHub:

    def __init__(self, read_fn, fps=10, name="framehub"):
        """
        read_fn: callable returning (ok, frame). Supplied by the server so the
                 hub does not need to know how the capture is locked or opened.
        """
        self._read = read_fn
        self._period = 1.0 / max(1, fps)
        self._name = name

        self._lock = threading.Condition()
        self._frame = None
        self._seq = 0
        self._stamp = 0.0

        self._running = False
        self._thread = None
        self._consumers = 0
        self.errors = 0

    # ── lifecycle ─────────────────────────────────────────────────────

    def start(self):
        with self._lock:
            if self._running:
                return
            self._running = True
        self._thread = threading.Thread(target=self._grab_loop, daemon=True,
                                        name=self._name)
        self._thread.start()

    def stop(self):
        with self._lock:
            self._running = False
            self._lock.notify_all()

    @property
    def running(self):
        return self._running

    def _grab_loop(self):
        misses = 0
        while self._running:
            t0 = time.time()
            try:
                ok, frame = self._read()
            except Exception:
                ok, frame = False, None

            if ok and frame is not None:
                misses = 0
                with self._lock:
                    self._frame = frame
                    self._seq += 1
                    self._stamp = t0
                    self._lock.notify_all()
            else:
                misses += 1
                self.errors += 1
                if misses >= 20:
                    print("[framehub] 20 consecutive read failures, stopping grabber")
                    break

            slack = self._period - (time.time() - t0)
            if slack > 0:
                time.sleep(slack)

        with self._lock:
            self._running = False
            self._lock.notify_all()

    # ── consumer API ──────────────────────────────────────────────────

    def latest(self):
        """(seq, frame, stamp) without waiting. seq==0 means nothing grabbed yet."""
        with self._lock:
            return self._seq, self._frame, self._stamp

    def wait_for_new(self, last_seq, timeout=2.0):
        """
        Block until a frame newer than last_seq arrives.
        Returns (seq, frame, stamp), or (last_seq, None, 0.0) on timeout/stop.
        """
        deadline = time.time() + timeout
        with self._lock:
            while self._running and self._seq <= last_seq:
                remaining = deadline - time.time()
                if remaining <= 0:
                    return last_seq, None, 0.0
                self._lock.wait(remaining)
            if self._seq > last_seq:
                return self._seq, self._frame, self._stamp
            return last_seq, None, 0.0
