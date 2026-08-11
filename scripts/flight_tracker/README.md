# flight_tracker

Per-arm flight recording for the GPS-denied airframe. Each arm → disarm cycle
becomes one flight: a row in `flights`, a 5 Hz track in `track_points`, timeline
markers in `flight_events`, and an mp4 on the SD card.

Served over REST + Socket.IO, so the webapp, a Windows/Mac build and a mobile
client all consume one implementation.

---

## Why local NED and not lat/lon

The aircraft has no usable GPS fix indoors, so `lat`/`lon` are meaningless.
Position comes from `LOCAL_POSITION_NED` — verified working on this airframe:

```
local: LocationLocal:north=0.0019, east=0.0112, down=-0.0282
vel  : [0.0, 0.0, 0.0]
rf   : 0.13
```

Stored per point, in metres:

| column | meaning |
|---|---|
| `x_m` | east, relative to the arm point |
| `y_m` | north, relative to the arm point |
| `z_m` | **up-positive** altitude (NED `down` is negated on the way in) |
| `pos_source` | `ekf` / `deadreckon` / `none` |

`local_frame` is measured from the **EKF origin**, which is set once when the
EKF initialises and then persists across arm/disarm cycles in the same power
session. The raw reading at the second arm of a session is therefore *not*
zero. The reading at arm is saved as `flights.origin_*` and subtracted from
every point — that is what makes each arm an independent track starting at
(0, 0, 0).

If `local_frame` ever stops populating, `LiveSource` falls back to integrating
`vehicle.velocity` and marks those points `deadreckon`. Dead reckoning drifts,
so the UI must render it as degraded rather than as a fix.

---

## Layout

```
flight_tracker/
  __init__.py     exports
  store.py        SQLite schema + queries
  sources.py      LiveSource (DroneKit) / SimSource (synthetic)
  recorder.py     FlightTracker — the arm-edge watcher and sample loop
  frames.py       FrameHub — single-reader camera fan-out
  video.py        VideoRecorder — ffmpeg/libx264, cv2 fallback
  routes.py       Flask blueprint mounted at /flights
```

Files land in `drone/flights/`:

```
flights/
  flights.db          SQLite (WAL — flights.db-wal / -shm alongside it)
  video/
    flight_00007_20260811_204133.mp4
```

---

## Server wiring

Already applied to `new_api_server_1.py`. For reference:

```python
from flight_tracker import FlightTracker, FrameHub, SAMPLE_HZ as TRACK_HZ
from flight_tracker.routes import make_blueprint as make_flights_bp

frame_hub = FrameHub(_hub_read, fps=CAMERA_FPS)

tracker = FlightTracker(db_path=FLIGHTS_DB, video_dir=FLIGHTS_VIDEO,
                        socketio=socketio, record_video=True)
app.register_blueprint(make_flights_bp(tracker), url_prefix="/flights")

# inside connect_drone(), after mc exists:
tracker.attach(vehicle, mc, frame_hub, start_camera)
tracker.start()
```

Import failure degrades gracefully: `[tracker] DISABLED — <error>` and the
server starts anyway, same as the WebRTC blueprint.

---

## The camera change

`cv2.VideoCapture.read()` **consumes** a frame. Two readers split the stream and
both run at half rate — which is why two browser tabs on `/camera/stream`
already stutter today.

`FrameHub` makes exactly one thread call `read()`. Everyone else takes the
latest frame from a slot:

```
/dev/video0 → VideoCapture → FrameHub (1 grab thread)
                                ├──► generate_frames() → MJPEG → browser
                                └──► VideoRecorder     → ffmpeg → SD card
```

`/camera/stream` keeps the same URL and output. Video recording does **not**
depend on anyone watching the stream — the recorder calls `start_camera()`
itself on arm. `/camera/stop` returns **409** while a recording is in progress
rather than truncating the file.

---

## Video encoding

Ladder, best first:

1. **ffmpeg → libx264 → .mp4** — browser-playable, ~800 kbps at 640×480/10 fps
   (~60 MB for 10 min). Runs as a **subprocess**, so the encode stays off the
   GIL that the 20 Hz RC override thread needs. The Pi 5 has no hardware H.264
   encoder (the Pi 4 did), so this is CPU work either way — the subprocess is
   what keeps it from competing with control.
2. `cv2` `mp4v` → .mp4 — always available, but **Chrome will not play it**.
   Archive only; transcode before browser replay.
3. `cv2` `MJPG` → .avi — last resort, large.

Which one was used is recorded in `flights.video_codec`.

`ffmpeg` needed for the good path: `sudo apt install ffmpeg`

---

## API

| method | path | notes |
|---|---|---|
| GET | `/flights` | `?limit=50&offset=0` |
| GET | `/flights/current` | live recording status |
| GET | `/flights/<id>` | metadata + summary + events |
| GET | `/flights/<id>/track` | `?fields=x_m,y_m,z_m&decimate=N&from=&to=` |
| GET | `/flights/<id>/events` | timeline markers |
| GET | `/flights/<id>/video` | mp4, HTTP Range → seekable |
| POST | `/flights/<id>/label` | `{label, notes}` |
| POST | `/flights/<id>/mark` | `{detail}` — marker on the live timeline |
| DELETE | `/flights/<id>` | `?keep_video=1` |
| GET | `/flights/stats` | counts, disk usage |
| POST | `/flights/sim/start` | `{alt, side, speed, hover_s}` |
| POST | `/flights/sim/stop` | |

Socket events: `flight_started`, `track_point` (5 Hz), `flight_ended`,
`tracker_status` (on connect).

### Track payload is column-oriented

```json
{"count": 3000,
 "columns": ["t_ms", "x_m", "y_m", "z_m"],
 "data": {"t_ms": [0,200,400], "x_m": [0,0.1,0.2], ...}}
```

Roughly a third of the bytes of a list-of-objects, because key names are not
repeated 3000 times. Use `decimate` for zoomed-out overviews and `fields` to
fetch only the columns a given chart needs.

---

## Simulator

No Pixhawk required. Produces identical DB rows, socket events and REST
payloads to a real flight, so the entire UI can be built and demoed without
waiting for space to fly.

```bash
curl -X POST localhost:5000/flights/sim/start \
     -H 'Content-Type: application/json' \
     -d '{"alt":2.5,"side":8,"speed":1.2}'
```

Profile: arm → climb → hover → 8 m square → hover → descend → disarm.
Attitude and RC channels are derived from the commanded motion, so the charts
and the corrector-vs-attitude comparison have plausible data to render.

---

## The corrector record

`track_points` stores the commanded RC channels **and** the closed-loop
correction delta next to actual attitude:

```
rc_roll  rc_pitch  rc_throttle  rc_yaw    ← what the Pi transmitted
corr_roll  corr_pitch                     ← what the corrector added
roll  pitch  yaw                          ← what the aircraft did
```

This makes the tracker a bench tool, not just a breadcrumb trail. Props off,
arm in LOITER, tilt the airframe by hand, then read the replay:

> nose-up tilt (`pitch` > 0) should drive `corr_pitch` **negative**.

If it drives positive, `PITCH_CORR_SIGN` in `movement_controller.py` is
inverted. That answers the open `RC2_REVERSED` question without flying.

---

## Safety properties

- **Passive.** Reads `vehicle` and `mc._rc` / `mc._corr`. Never writes to
  either. A bug here cannot move the aircraft.
- **Never blocks flight.** All DB writes batched (1 s) and every exception
  caught and logged. Losing history is acceptable; stalling a thread that
  shares a process with the RC override is not.
- **Crash-safe.** A flight with no `ended_at` is closed as `interrupted` at the
  next startup, so a power-cut mid-flight does not leave a row the UI renders
  as "still flying" forever.
- **SD-card friendly.** `synchronous=NORMAL` + batched commits.

---

## Retention

None. Nothing is auto-deleted — by request. `GET /flights/stats` reports
`db_bytes` and `video_bytes` so growth is visible.

Rough sizes: **track** ~3000 rows ≈ 700 KB per 10-min flight (negligible).
**Video** dominates — ~60 MB per 10 min at the default bitrate. Watch
`video_bytes` and prune manually via `DELETE /flights/<id>` until a retention
policy is added.

---

## Not yet done

- UI (next step): live 2D path, scrubber, synced charts, video sync.
- Cloud upload on disarm — `flights.uploaded_at` is reserved for it.
- Object detection overlay — deferred by request.
