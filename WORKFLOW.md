# Drone Pi 2 — Full System Workflow

## Hardware

| Component | Detail |
|-----------|--------|
| Companion computer | Raspberry Pi 2 |
| Flight controller | Pixhawk (ArduCopter, EKF3) |
| Serial link | `/dev/ttyAMA0` — UART between Pi 2 and Pixhawk, baud 57600 |
| Optical flow | MTF-01 (`FLOW_TYPE=5` MAVLink) |
| Rangefinder | Connected via MAVLink (`RNGFND_TYPE=10`) |
| GPS | None — `EK3_SRC1_POSXY=0`, `EK3_SRC1_VELXY=5` (optical flow), `EK3_SRC1_POSZ=2` (rangefinder) |
| Battery | 4S LiPo (min safe voltage: 13.2V = 3.3V/cell) |
| Camera | USB camera on `/dev/video0` (V4L2, MJPG, 640×480, 10 FPS) |

---

## Software Stack

```
Python 3 (venv at /home/pi/drone/venv)
├── Flask 3.1.3             — HTTP REST API
├── Flask-SocketIO 5.6.1    — WebSocket server
├── Flask-CORS 6.0.2        — Cross-origin support
├── eventlet 0.40.4         — Async I/O / monkey-patch
├── DroneKit 2.9.2          — High-level MAVLink vehicle abstraction
├── pymavlink 2.4.49        — Low-level MAVLink message building
├── opencv-python-headless  — Camera capture + JPEG encode
└── subprocess (stdlib)     — nmcli WiFi + ngrok process control
```

---

## Boot Sequence (systemd)

```
Power on Pi
     │
     ▼
NetworkManager.service
     │
     ▼
wifi-manager.service  (Type=oneshot, RemainAfterExit=yes)
     │   Runs: /home/pi/wifi_manager.sh
     │   Waits 20s for any WiFi to connect
     │   If connected → exits 0 (active/exited = normal)
     │   If no WiFi   → nmcli connection up "DroneAP" (AP hotspot)
     │                  SSID: DroneAP, IP: 192.168.4.1, WPA2
     │
     ▼
drone-api.service  (Restart=always, RestartSec=5)
     │   After=wifi-manager.service network.target
     │   User=pi
     │   WorkingDirectory=/home/pi/drone
     │   ExecStart=venv/bin/python3 scripts/new_api_server_1.py
     │
     ▼
new_api_server_1.py __main__ startup sequence:
     1. scan_networks()              — nmcli rescan, 2s settle
     2. connect_wifi("dynamics")     — skip if already connected
     3. wifi_connection_retries()    — up to 50 retries × 5s
     4. wifi_monitor_loop() daemon   — background thread, checks every 15s
     5. time.sleep(5)               — settle
     6. start_ngrok()               — subprocess.Popen ngrok http 5000
     7. connect_drone()             — DroneKit connect /dev/ttyAMA0
     8. socketio.run(host=0.0.0.0, port=5000)
```

---

## WiFi Management

### Boot script — `wifi_manager.sh`

```
On boot:
  Poll nmcli STATE every 1s for 20s
  → "connected" found: log IP, exit 0
  → timeout:          nmcli connection up "DroneAP"
```

### Runtime management — `new_api_server_1.py`

| Function | What it does |
|----------|-------------|
| `scan_networks()` | `nmcli device wifi rescan`, sleeps 2s |
| `is_ssid_available(ssid)` | `nmcli -t -f SSID dev wifi list`, checks target in list |
| `get_active_ssid()` | `nmcli -t -f ACTIVE,SSID device wifi`, returns connected SSID or None |
| `is_wifi_connected()` | `nmcli -t -f STATE general`, checks "connected" in output |
| `connect_wifi(ssid, pw)` | Check already connected first → skip. Else `sudo nmcli device wifi connect <ssid> password <pw>` |
| `wifi_connection_retries()` | Startup loop: 50 attempts, 5s delay, scan+connect each |
| `wifi_monitor_loop()` | Daemon thread: every 15s check `is_wifi_connected()`, reconnect if dropped |

**sudo required**: `nmcli device wifi connect` needs root. Pi user granted via:
```
/etc/sudoers.d/pi-nmcli:
  pi ALL=(ALL) NOPASSWD: /usr/bin/nmcli
```

---

## MAVLink / DroneKit Communication

### Physical path

```
Pixhawk ──UART──► /dev/ttyAMA0 ──► DroneKit (Python) ──► Flask API
  ▲                                      │
  └──────────── MAVLink messages ◄───────┘
```

### Connection

```python
vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=False)
```

`wait_ready=False` — don't block on parameter fetch; Pi 2 is slow.

### MAVLink message types used

| Message | MAVLink ID | Usage |
|---------|-----------|-------|
| `SET_POSITION_TARGET_LOCAL_NED` | #84 | All velocity commands (move, hold stop) |
| `SET_ATTITUDE_TARGET` | #82 | GPS-less takeoff via thrust control |
| `COMMAND_LONG` → `MAV_CMD_CONDITION_YAW` | #178 | Yaw rotation |
| `COMMAND_LONG` → `MAV_CMD_PREFLIGHT_CALIBRATION` | #241 | Level calibration |
| `COMMAND_LONG` → `MAV_CMD_DO_SET_SAFETY_SWITCH` | #223 | Safety switch on/off |

### `SET_POSITION_TARGET_LOCAL_NED` frame

```
Frame:    MAV_FRAME_BODY_NED (frame=8)
Type mask: 0b0000111111000111  → velocity only (ignore pos, accel, yaw)

Axes (body-relative):
  vx = +forward / −backward
  vy = +right   / −left
  vz = +down    / −up   (NED convention, negative = climb)
```

**Critical**: these commands are silently ignored in ALTHOLD/STABILIZE.
Only work in `GUIDED` or `GUIDED_NOGPS`.

---

## Flight Mode Logic

### Mode selection at takeoff

```python
if vehicle.gps_0.fix_type < 3:
    vehicle.mode = VehicleMode("GUIDED_NOGPS")   # optical flow + rangefinder
else:
    vehicle.mode = VehicleMode("GUIDED")          # GPS
```

### GPS-less takeoff (GUIDED_NOGPS)

```
No GPS → arm → SET_ATTITUDE_TARGET thrust loop
  hover_thrust = 0.3732573  (matches MOT_THST_HOVER param)
  climb_thrust = hover_thrust + 0.10
  Loop at 10 Hz: send climb_thrust, read rangefinder.distance
  Stop when rangefinder >= target_alt × 0.95
  Transition: send hover_thrust × 20 cycles to stabilise
  Timeout: 30s → LAND mode
```

### GPS takeoff

```
GPS fix ≥ 3 → vehicle.simple_takeoff(altitude)
  Wait until global_relative_frame.alt >= altitude × 0.95
```

### Move endpoint — mode enforcement

```python
if vehicle.mode.name not in ('GUIDED', 'GUIDED_NOGPS'):
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    time.sleep(0.3)
# then send velocity command
```

### Hold endpoint — position hold logic

```python
if GPS fix >= 3:   switch to LOITER (GPS holds XY + alt)
else:              stay GUIDED_NOGPS + send zero-velocity stop
                   (optical flow holds XY, rangefinder holds Z)
```

### EKF3 sensor configuration (no GPS)

| EKF parameter | Value | Meaning |
|--------------|-------|---------|
| `EK3_SRC1_POSXY` | 0 | No absolute XY position source |
| `EK3_SRC1_VELXY` | 5 | Optical flow (MTF-01) |
| `EK3_SRC1_POSZ` | 2 | Rangefinder |
| `EK3_SRC1_YAW` | 1 | Compass |

EKF healthy state: `pos_horiz_rel: On`, `pos_vert_agl: On`, `pos_horiz_abs: Off`.

---

## Concurrency Model

```
Main thread:   socketio.run() — Flask request handling (eventlet)
               └─ eventlet.monkey_patch() at top of file

Daemon threads:
  stream_telemetry()     — emits via socketio every 200ms
  arm_and_takeoff()      — holds _cmd_lock, releases in finally
  _send_velocity_locked() — holds _cmd_lock, releases in finally
  _send_yaw_locked()     — holds _cmd_lock, releases in finally
  _arm_drone()           — holds _cmd_lock, releases in finally
  wifi_monitor_loop()    — WiFi watchdog every 15s
```

### Command lock

```python
_cmd_lock = threading.Lock()
```

- Non-blocking acquire (`blocking=False`) at every flight endpoint
- Returns HTTP 409 if another command is already running
- Lock always released in `finally` block — no deadlock on exception
- Emergency endpoint intentionally bypasses lock

---

## All REST API Endpoints

| Method | Path | Body / Params | What happens |
|--------|------|--------------|-------------|
| `GET` | `/` | — | Serve `templates/index.html` |
| `GET` | `/health` | — | Connection + armed + mode + battery + GPS fix |
| `GET` | `/status` | — | Full telemetry snapshot (18 fields) |
| `POST` | `/lvlcal` | — | `MAV_CMD_PREFLIGHT_CALIBRATION` param5=2 (level cal) |
| `POST` | `/arm` | — | Arms vehicle (daemon thread, holds lock) |
| `POST` | `/disarm` | — | Disarms vehicle |
| `POST` | `/safety` | `{state: "on"\|"off"}` | `MAV_CMD_DO_SET_SAFETY_SWITCH` |
| `POST` | `/takeoff` | `{altitude: 2}` | Battery check → mode set → arm → thrust/GPS takeoff |
| `POST` | `/land` | — | `VehicleMode("LAND")` |
| `POST` | `/rtl` | — | GPS required → `VehicleMode("RTL")` |
| `POST` | `/hold` | — | LOITER (GPS) or zero-velocity GUIDED_NOGPS |
| `POST` | `/move` | `{direction, distance, speed}` | Enforce GUIDED_NOGPS → velocity for `distance/speed` seconds |
| `POST` | `/yaw` | `{direction, degrees, speed}` | `MAV_CMD_CONDITION_YAW` |
| `POST` | `/mode` | `{mode: "STABILIZE"\|...}` | Set ArduCopter flight mode |
| `GET` | `/param` | `?name=ARMING_CHECK` | Read single parameter |
| `POST` | `/param` | `{param, value}` | Write single parameter |
| `POST` | `/emergency` | — | `LAND` → disarm (bypasses command lock) |
| `GET` | `/video_feed` | — | MJPEG multipart stream |
| `GET/POST` | `/camera/start` | — | Open `/dev/video0` via V4L2 |
| `GET/POST` | `/camera/stop` | — | Release camera |
| `GET` | `/wifi/scan` | — | `nmcli` list with SSID/signal/security |
| `POST` | `/wifi/connect` | `{ssid, password}` | `sudo nmcli device wifi connect` |
| `POST` | `/queue/add` | `{action, params}` | Append to `_cmd_queue` deque (not yet auto-executed) |
| `GET` | `/queue/status` | — | Current queue depth + items |
| `POST` | `/queue/clear` | — | Empty queue |

### Endpoint validation highlights

- `/takeoff`: battery < 13.2V → 400; altitude > 10m → 400; altitude < 0.5m → 400
- `/move`: speed > 1.5 m/s → 400 (optical flow limit); invalid direction → 400
- `/yaw`: degrees > 360 → 400; direction not left/right → 400
- `/rtl`: GPS fix < 3 → 400
- All flight endpoints: lock busy → 409 Conflict

---

## WebSocket Telemetry

### Protocol

```
Browser ──SocketIO/WS──► Flask-SocketIO (eventlet)
                              │
                         stream_telemetry() daemon thread
                         emits 'telemetry' event every 200ms
```

### Telemetry payload (18 fields)

```json
{
  "mode":          "GUIDED_NOGPS",
  "armed":         false,
  "rangefinder":   1.23,
  "altitude":      1.2,
  "battery":       15.1,
  "battery_level": 87,
  "satellites":    0,
  "gps_fix":       0,
  "pitch":         0.012345,
  "roll":          -0.003456,
  "yaw":           1.570796,
  "lat":           0.0,
  "lon":           0.0,
  "groundspeed":   0.0,
  "airspeed":      0.0,
  "ekf_ok":        true,
  "is_armable":    true
}
```

One telemetry thread starts on first WebSocket client connect, never restarts (`_telemetry_started` flag + `_telemetry_lock`).

---

## Camera Streaming

```
/dev/video0 (V4L2)
     │  640×480, MJPG codec, 10 FPS
     ▼
cv2.VideoCapture → cap.read()
     │  cv2.imencode('.jpg', frame, JPEG quality=70%)
     ▼
multipart/x-mixed-replace HTTP stream  ← /video_feed endpoint
     │
     ▼
Browser <img src="/video_feed">
```

Thread safety: `camera_lock` (threading.Lock) guards `camera` global.
Double-check pattern: check outside lock (fast), open camera, re-check inside lock (safe).
Error tolerance: 10 consecutive read failures → stop stream.
FPS pacing: `socketio.sleep(1.0 / CAMERA_FPS)` — cooperates with eventlet.

---

## ngrok Tunnel

```
socketio.run(host=0.0.0.0, port=5000)
     ▲
     │  local HTTP
     │
ngrok http 5000  (subprocess.Popen, stdout/stderr suppressed)
     │
     ▼
https://<random>.ngrok-free.app  ← public URL
     ▲
     │
Browser (anywhere) or LLM integration
```

ngrok starts after WiFi settles (5s delay). URL must be fetched from ngrok API or ngrok dashboard — not auto-printed.

---

## Control Website (`drone_control_website/`)

| File | Purpose |
|------|---------|
| `index.html` | Dashboard — telemetry display |
| `control.html` | Gamepad controller — directional + yaw + altitude |
| `settings.html` | Server URL config, flight params |
| `docs.html` | API reference |
| `static/css/main.css` | Shared dark UI styles |
| `static/js/core.js` | `Drone` object — URL management, `api()` fetch wrapper, SocketIO telemetry, log panel |

### Control page features

- **D-Pad**: forward/backward/left/right → `POST /move`
- **Altitude pad**: up/down → `POST /move`
- **Yaw buttons**: left/right rotation → `POST /yaw`
- **Hold-to-move**: press + hold = continuous repeat at `distance/speed * 1000ms + 200ms` intervals; release = `POST /hold`
- **Speed input**: max 1.5 m/s (optical flow limit); default 0.5; saved to localStorage
- **Keyboard**: WASD + arrow keys + Q/E (yaw) + R/F (altitude)
- **Quick actions**: Hold / RTL / Land / Emergency Stop
- **Connection badge**: SocketIO connect/disconnect state

### SocketIO in browser

```js
const socket = io();   // auto-connects to same origin (ngrok URL)
socket.on('telemetry', data => {
    // update pills: mode, armed, altitude, battery, GPS
    // yaw converted: radians → degrees, normalised 0–360
});
```

---

## Internal Templates (Pi-served UI)

`drone/templates/index.html` — telemetry-only display, served by Flask at `/`.
Shows: MODE, ARMED, ALTITUDE, BATTERY, SATELLITES, GPS FIX, PITCH, ROLL, YAW, VIBE X/Y/Z, GROUNDSPEED, LAT, LON.
Connects via `const socket = io()` to same Flask server.

---

## Key Fixes Applied This Project

| Problem | Root cause | Fix |
|---------|-----------|-----|
| Move commands silently ignored | `SET_POSITION_TARGET_LOCAL_NED` only works in GUIDED/GUIDED_NOGPS; ALTHOLD discards it | `/move` auto-switches to GUIDED_NOGPS before sending velocity |
| `/hold` caused XY drift | Switched to ALTHOLD: baro holds Z but no XY hold | `/hold` stays GUIDED_NOGPS + sends zero velocity; optical flow holds XY |
| Battery check blocked 4S takeoff | Threshold was 10.5V (3S minimum); 4S needs 13.2V | Changed threshold to `< 13.2` |
| "Insufficient privileges" on WiFi connect | `nmcli device wifi connect` needs root; service runs as `pi` | Added `sudo` prefix; granted `pi` passwordless sudo for nmcli |
| Reconnect attempt when already on target WiFi | No pre-check before `nmcli connect` | `get_active_ssid()` check at start of `connect_wifi()` — skips if match |
| WiFi drop required reboot | No runtime reconnect logic | `wifi_monitor_loop()` daemon thread polls every 15s, reconnects if dropped |
| Hotspot name mismatch | `HOTSPOT_CON="DroneControl"` but NM connection named `"DroneAP"` | Fixed to `HOTSPOT_CON="DroneAP"` in `wifi_manager.sh` |
| Multiple telemetry threads on reconnect | Thread spawned on every WebSocket `connect` event | `_telemetry_started` flag + `_telemetry_lock` — thread created once only |
| Crash on None DroneKit fields | `vehicle.battery.voltage` etc. can return None | `safe_float()`, `safe_int()`, `safe_round()` helpers used everywhere |

---

## Data Flow Summary

```
                    ┌─────────────────────────────────────┐
                    │         Raspberry Pi 2              │
                    │                                     │
Internet ──ngrok──► │ :5000 Flask+SocketIO (eventlet)     │
                    │   │                                 │
  Browser ──WiFi──► │   ├── REST endpoints (/move etc.)  │
                    │   └── WebSocket (/socket.io)        │
                    │         │ telemetry 5Hz             │
                    │         ▼                           │
                    │   DroneKit vehicle object           │
                    │         │ MAVLink over              │
                    │         ▼ /dev/ttyAMA0 57600        │
                    └─────────┼───────────────────────────┘
                              │ UART
                              ▼
                         Pixhawk (ArduCopter EKF3)
                              │
                    ┌─────────┴───────────────────┐
                    │  MTF-01 optical flow         │ MAVLink
                    │  Rangefinder                 │ MAVLink
                    │  Compass (internal)          │
                    │  IMU/Baro (internal)         │
                    └──────────────────────────────┘
```
