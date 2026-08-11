"""
Per-flight video recording, stored locally on the Pi.

Encoder selection ladder, best first:

  1. ffmpeg subprocess, libx264 -> .mp4
     Browser-playable, small (~600 kbps at 640x480/10fps is ~45 MB for 10 min),
     and the encode runs in a SEPARATE PROCESS. That last point matters: the
     Pi 5 has no hardware H.264 encoder (the Pi 4 did), so encoding is CPU
     work, and a subprocess keeps it off the GIL that the 20 Hz RC override
     thread is competing for.

  2. cv2.VideoWriter with mp4v -> .mp4
     Always available, but MPEG-4 Part 2 does NOT play in Chrome. Usable as an
     archive, needs transcoding before browser replay.

  3. cv2.VideoWriter with MJPG -> .avi
     Last resort. Large, but it will always write something.

The recorder never blocks the frame hub: it pulls the latest frame at a fixed
rate and duplicates the previous frame if the camera stalls, so wall-clock time
and video time stay aligned. That alignment is what lets the scrubber seek the
video by t_ms.
"""

import os
import shutil
import subprocess
import threading
import time

try:
    import cv2
except Exception:      # pragma: no cover - camera-less dev machine
    cv2 = None


def ffmpeg_available():
    return shutil.which("ffmpeg") is not None


class VideoRecorder:
    """
    Records from a FrameHub to one file. Start/stop is driven by the flight
    recorder so the file lifetime matches the flight exactly.
    """

    def __init__(self, hub, out_dir, fps=10, size=(640, 480), bitrate="800k",
                 prefer_ffmpeg=True):
        self.hub = hub
        self.out_dir = out_dir
        self.fps = int(fps)
        self.width, self.height = size
        self.bitrate = bitrate
        self.prefer_ffmpeg = prefer_ffmpeg

        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self.path = None
        self.codec = None
        self.started_at = None      # wall clock when the first frame was written
        self.frames = 0
        self.error = None

    @property
    def active(self):
        return self._thread is not None and self._thread.is_alive()

    # ── control ───────────────────────────────────────────────────────

    def start(self, basename):
        with self._lock:
            if self.active:
                return None
            os.makedirs(self.out_dir, exist_ok=True)
            self._stop.clear()
            self.frames = 0
            self.error = None
            self.started_at = None

            use_ff = self.prefer_ffmpeg and ffmpeg_available()
            ext = ".mp4" if (use_ff or cv2 is not None) else ".avi"
            self.path = os.path.join(self.out_dir, basename + ext)

            self._thread = threading.Thread(
                target=self._run, args=(use_ff,), daemon=True,
                name="video-recorder")
            self._thread.start()
            return self.path

    def stop(self, timeout=8.0):
        """Stop and return a dict of results for the DB row."""
        self._stop.set()
        t = self._thread
        if t is not None:
            t.join(timeout=timeout)
        self._thread = None

        size = None
        try:
            if self.path and os.path.exists(self.path):
                size = os.path.getsize(self.path)
        except Exception:
            pass

        return {
            "path": self.path,
            "codec": self.codec,
            "frames": self.frames,
            "duration_s": round(self.frames / self.fps, 2) if self.frames else 0.0,
            "size_bytes": size,
            "started_at": self.started_at,
            "error": self.error,
        }

    # ── worker ────────────────────────────────────────────────────────

    def _open_ffmpeg(self):
        cmd = [
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-s", f"{self.width}x{self.height}",
            "-r", str(self.fps),
            "-i", "-",
            "-an",
            "-c:v", "libx264",
            "-preset", "ultrafast",     # cheapest CPU; the control loop has priority
            "-tune", "zerolatency",
            "-pix_fmt", "yuv420p",      # required for browser playback
            "-b:v", self.bitrate,
            "-movflags", "+faststart",
            self.path,
        ]
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        self.codec = "h264"
        return proc

    def _open_cv(self):
        if cv2 is None:
            raise RuntimeError("cv2 unavailable")
        for fourcc, ext, name in (("mp4v", ".mp4", "mp4v"), ("MJPG", ".avi", "mjpg")):
            path = os.path.splitext(self.path)[0] + ext
            w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc),
                                self.fps, (self.width, self.height))
            if w.isOpened():
                self.path = path
                self.codec = name
                return w
            w.release()
        raise RuntimeError("no usable cv2 encoder")

    def _run(self, use_ffmpeg):
        proc = writer = None
        try:
            if use_ffmpeg:
                proc = self._open_ffmpeg()
            else:
                writer = self._open_cv()
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"
            print(f"[video] encoder open failed: {self.error}")
            try:
                if not use_ffmpeg:
                    return
                writer = self._open_cv()
                proc = None
            except Exception as e2:
                self.error = f"{self.error}; fallback: {e2}"
                return

        period = 1.0 / self.fps
        last_seq = 0
        last_frame = None
        next_t = time.time()

        while not self._stop.is_set():
            seq, frame, _ = self.hub.latest()
            if frame is not None and seq != last_seq:
                last_seq = seq
                last_frame = frame

            if last_frame is not None:
                out = last_frame
                if cv2 is not None and (out.shape[1] != self.width or
                                        out.shape[0] != self.height):
                    out = cv2.resize(out, (self.width, self.height))
                try:
                    if proc is not None:
                        proc.stdin.write(out.tobytes())
                    else:
                        writer.write(out)
                    if self.started_at is None:
                        self.started_at = time.time()
                    self.frames += 1
                except Exception as e:
                    self.error = f"write failed: {type(e).__name__}: {e}"
                    print(f"[video] {self.error}")
                    break

            next_t += period
            slack = next_t - time.time()
            if slack > 0:
                time.sleep(slack)
            else:
                # Fell behind (SD card stall). Resync rather than accumulating
                # debt and then busy-spinning to catch up.
                next_t = time.time()

        try:
            if proc is not None:
                try:
                    proc.stdin.close()
                except Exception:
                    pass
                proc.wait(timeout=6)
            elif writer is not None:
                writer.release()
        except Exception:
            try:
                if proc is not None:
                    proc.kill()
            except Exception:
                pass
        print(f"[video] closed {self.path} frames={self.frames} codec={self.codec}")
