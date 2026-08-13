import collections
import os
import json
#import eventlet
#eventlet.monkey_patch()

from flask import Flask, request, jsonify, render_template, Response
from flask_socketio import SocketIO
from flask_cors import CORS
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import threading
import time
import cv2
import subprocess
# WebRTC is optional: import failure must not take the whole server down.
# This was `from scripts.webrtc.routes import bp` — but the service runs
# `python3 /home/pi/drone/scripts/new_api_server_1.py`, so sys.path[0] is the
# scripts/ directory itself and there is no importable `scripts` package.
# `webrtc` is the correct top-level name from there. aiortc is also missing
# from requirements.txt, so a clean venv fails on that instead.
try:
    from webrtc.routes import bp as webrtc_bp
    WEBRTC_AVAILABLE = True
    WEBRTC_ERROR = None
except Exception as _e:            # ImportError, or aiortc/av not installed
    webrtc_bp = None
    WEBRTC_AVAILABLE = False
    WEBRTC_ERROR = f"{type(_e).__name__}: {_e}"

from movement_controller import MovementController, CORR_RATE_HZ, RC_RATE_HZ
from mission_manager import MissionManager

# Flight tracker is optional for the same reason WebRTC is: a missing dependency
# or a corrupt DB must not stop the aircraft's control server from starting.
try:
    from flight_tracker import FlightTracker, FrameHub, SAMPLE_HZ as TRACK_HZ
    from flight_tracker.routes import make_blueprint as make_flights_bp
    TRACKER_AVAILABLE = True
    TRACKER_ERROR = None
except Exception as _e:
    FlightTracker = None
    FrameHub = None
    TRACK_HZ = 0
    TRACKER_AVAILABLE = False
    TRACKER_ERROR = f"{type(_e).__name__}: {_e}"

# Click-to-waypoint navigation, optional on the same principle.
try:
    from waypoint_nav import (WaypointNav, make_nav_bp, NAV_DEADMAN_S,
                              MAX_RADIUS_M as NAV_MAX_RADIUS)
    NAV_AVAILABLE = True
    NAV_ERROR = None
except Exception as _e:
    WaypointNav = None
    NAV_DEADMAN_S = 0.3
    NAV_MAX_RADIUS = 0.0
    NAV_AVAILABLE = False
    NAV_ERROR = f"{type(_e).__name__}: {_e}"

def scan_networks():
    print("Triggering Wi-Fi environment rescan...")
    subprocess.run(
	['/usr/bin/nmcli','device','wifi','rescan'],
	capture_output=True,
	text=True
	)

    time.sleep(2)
    
    # result = subprocess.run(
	# ['nmcli','-t','-f','SSID,SIGNAL','device', 'wifi', 'list'],
	# capture_output=True,
	# test=True
	# )

    # for line in result.stdout.splitlines():
    #     print(line)

def is_ssid_available(target_ssid):
    """Checks the current visibility list for the target SSID."""
    cmd = ['/usr/bin/nmcli', '-t', '-f', 'SSID', 'dev', 'wifi', 'list']
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    # Create a clean list of visible SSIDs
    visible_ssids = [line.strip() for line in result.stdout.split('\n') if line.strip()]
    return target_ssid in visible_ssids

def wifi_connection_retries(ssid,password=None):
    MAX_RETRIES = 5
    RETRY_DELAY = 5 # seconds to wait before trying to scan again
    
    connected = False

    for attempt in range(1, MAX_RETRIES + 1):
        print(f"\n--- Wi-Fi Connection Attempt [{attempt}/{MAX_RETRIES}] ---")
        
        # 1. Scan the airspace
        scan_networks()
        
        # 2. Check if our target drone network was caught in the net
        if is_ssid_available(ssid):
            # 3. If it's there, try connecting
            if connect_wifi(ssid, password):
                print(f"SUCCESS: Connected to {ssid}!")
                connected = True
                break # Exit the loop, we are done!
            else:
                print("Connection refused (Wrong password or weak signal).")
        else:
            print(f"Target SSID '{ssid}' not detected in this scan area.")
            
        # If we didn't connect, wait before trying the next loop iteration
        if attempt < MAX_RETRIES:
            print(f"Waiting {RETRY_DELAY} seconds before trying next scan...")
            time.sleep(RETRY_DELAY)

    if not connected:
        print(f"\nFATAL: Could not connect to {ssid} after {MAX_RETRIES} attempts.")
        # Handle your drone failure state here (e.g., return to base, throw alert, etc.)
    


def connect_wifi(ssid, password=None):
    try:
        if get_active_ssid() == ssid:
            print(f"Already connected to '{ssid}', skipping connect.")
            return True

        print(f"Connecting to WiFi: {ssid}")
        cmd = ['sudo', '/usr/bin/nmcli', 'device', 'wifi', 'connect', ssid]
        if password:
            cmd += ['password', password]

        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

        if result.returncode == 0:
            print(f"Connected to {ssid}")
            return True
        else:
            print(f"WiFi Connection failed: {result.stderr}")
            return False

    except Exception as e:
        print(f"WiFi error: {e}")
        return False


def is_wifi_connected():
    """Returns True if wlan0 has an active IP (i.e. WiFi is up)."""
    result = subprocess.run(
        ['/usr/bin/nmcli', '-t', '-f', 'STATE', 'general'],
        capture_output=True, text=True
    )
    return 'connected' in result.stdout.lower()

def get_active_ssid():
    """Returns the SSID currently connected on wlan0, or None."""
    result = subprocess.run(
        ['/usr/bin/nmcli', '-t', '-f', 'ACTIVE,SSID', 'device', 'wifi'],
        capture_output=True, text=True
    )
    for line in result.stdout.splitlines():
        parts = line.split(':')
        if len(parts) >= 2 and parts[0] == 'yes':
            return parts[1]
    return None

def wifi_monitor_loop(ssid, password=None):
    """Daemon thread: checks WiFi every 15s, reconnects if dropped."""
    CHECK_INTERVAL = 15  # seconds between checks
    while True:
        time.sleep(CHECK_INTERVAL)
        if not is_wifi_connected():
            print(f"[wifi_monitor] Connection lost — attempting reconnect to '{ssid}'")
            scan_networks()
            if is_ssid_available(ssid):
                if connect_wifi(ssid, password):
                    print(f"[wifi_monitor] Reconnected to '{ssid}'")
                else:
                    print(f"[wifi_monitor] Reconnect failed — will retry in {CHECK_INTERVAL}s")
            else:
                print(f"[wifi_monitor] '{ssid}' not visible — will retry in {CHECK_INTERVAL}s")

def start_ngrok():
    try:
        process = subprocess.Popen(['ngrok','http','5000'],
		stdout=subprocess.DEVNULL,
		stderr=subprocess.DEVNULL )
       # process = subprocess.run(['ngrok','http','5000'])

        print("nrgok started")
        return process

    except Exception as e:
        print(f"failed to start ngrok: {e}")
        return None






# ─── Camera State ────────────────────────
camera             = None
camera_lock        = threading.Lock()
camera_active      = False
CAMERA_FPS         = 10
CAMERA_JPEG_QUALITY = 70   # 70% quality: fine for live stream, ~40% smaller than default

# Seconds of frames kept in memory so a flight video can start BEFORE the arm.
# The interesting part of an indoor flight is usually the takeoff, and recording
# only from the armed edge misses the run-up every time.
#
# Costs RAM: a 640x480 BGR frame is ~0.9 MB, so 3 s at 10 fps holds ~27 MB
# resident while the camera runs. Fine on a Pi 5; noticeable on a Pi 2's 1 GB.
# Set to 0 to disable.
CAMERA_PREROLL_S   = 3.0

# Start grabbing as soon as the server does, rather than waiting for the first
# viewer. Two reasons: the pre-roll buffer is only useful if the camera was
# already running when the aircraft armed, and the live stream then has frames
# to serve immediately instead of paying the ~1 s V4L2 settle on first request.
CAMERA_AUTOSTART   = True

# Exactly one thread may call camera.read() — read() CONSUMES a frame, so two
# readers split the stream between them and both run at half rate. The hub owns
# that single read and hands the latest frame to every consumer (the MJPEG
# generator, the flight video recorder, and any additional stream viewers).
def _hub_read():
    with camera_lock:
        cap = camera
    if cap is None:
        return False, None
    return cap.read()


frame_hub = (FrameHub(_hub_read, fps=CAMERA_FPS, preroll_s=CAMERA_PREROLL_S)
             if TRACKER_AVAILABLE else None)

# ─── Flight Tracker Paths ────────────────
# scripts/../flights/  →  /home/pi/drone/flights/
FLIGHTS_DIR   = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'flights')
FLIGHTS_DB    = os.path.join(FLIGHTS_DIR, 'flights.db')
FLIGHTS_VIDEO = os.path.join(FLIGHTS_DIR, 'video')

app = Flask(__name__,
            template_folder='../templates',
            static_folder='../static')

CORS(app)  # handles OPTIONS preflight responses

if WEBRTC_AVAILABLE:
    app.register_blueprint(
        webrtc_bp,
        url_prefix="/webrtc"      # routes declare /connect -> /webrtc/connect
    )
    print("[webrtc] blueprint registered at /webrtc")
else:
    print(f"[webrtc] DISABLED — {WEBRTC_ERROR}")

@app.after_request
def add_cors_headers(response):
    response.headers['Access-Control-Allow-Origin']  = '*'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type, ngrok-skip-browser-warning'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
    return response
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode='threading'
)

# ─── Flight Tracker ──────────────────────
# Created after socketio so it can push live 'track_point' / 'flight_started' /
# 'flight_ended' events. It is PASSIVE: it reads the vehicle and the
# MovementController and never writes to either, so it cannot affect flight.
tracker = None
if TRACKER_AVAILABLE:
    try:
        tracker = FlightTracker(
            db_path=FLIGHTS_DB,
            video_dir=FLIGHTS_VIDEO,
            socketio=socketio,
            record_video=True,
            video_fps=CAMERA_FPS,
        )
        app.register_blueprint(make_flights_bp(tracker), url_prefix="/flights")
        print(f"[tracker] blueprint registered at /flights  (db: {FLIGHTS_DB})")
    except Exception as _e:
        tracker = None
        TRACKER_AVAILABLE = False
        TRACKER_ERROR = f"{type(_e).__name__}: {_e}"
        print(f"[tracker] DISABLED — {TRACKER_ERROR}")
else:
    print(f"[tracker] DISABLED — {TRACKER_ERROR}")

# ─── Waypoint Navigation ─────────────────
# Unlike the tracker, this one WRITES to the aircraft — it is a closed position
# loop that drives the RC override stream. Every safety gate lives inside
# WaypointNav; the server's job is to hand it a pose, a way to send sticks, and
# to abort it whenever anything else takes control (see _nav_abort callers).
nav = None
if NAV_AVAILABLE:
    try:
        nav = WaypointNav(socketio=socketio, mode="rc")
        app.register_blueprint(make_nav_bp(nav), url_prefix="/nav")
        print("[nav] blueprint registered at /nav")
    except Exception as _e:
        nav = None
        NAV_AVAILABLE = False
        NAV_ERROR = f"{type(_e).__name__}: {_e}"
        print(f"[nav] DISABLED — {NAV_ERROR}")
else:
    print(f"[nav] DISABLED — {NAV_ERROR}")

# ─── Global State ────────────────────────
vehicle = None

# FIX 1: single telemetry thread guard
_telemetry_started = False
_telemetry_lock    = threading.Lock()

# FIX 5: command lock — one flight action at a time
_cmd_lock = threading.Lock()

# FIX 6: command queue skeleton (not yet executed automatically)
# Future: mission sequencer pops from this deque and runs commands sequentially.
# Each item: {"action": str, "params": dict}
_cmd_queue      = collections.deque()
_cmd_queue_lock = threading.Lock()

# takeoff cancel flag - set by land/hold/emergency to abort arm_and_takeoff loop
_takeoff_cancelled = False

# motion cancel flag - set by hold/emergency to abort send_velocity loop mid-move
_motion_cancelled = False

# movement controller and mission manager (initialised after drone connects)
mc = None
mm = None

# ─── PID Tuner Config ────────────────────
PID_SAVES_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'pid_saves'
)

PID_GROUPS = {
    "Rate Roll": {
        "ATC_RAT_RLL_P":    {"label": "P Gain",        "default": 0.135},
        "ATC_RAT_RLL_I":    {"label": "I Gain",        "default": 0.135},
        "ATC_RAT_RLL_D":    {"label": "D Gain",        "default": 0.0036},
        "ATC_RAT_RLL_IMAX": {"label": "I Max",         "default": 0.5},
        "ATC_RAT_RLL_FLTD": {"label": "D Filter (Hz)", "default": 20.0},
        "ATC_RAT_RLL_FLTT": {"label": "T Filter (Hz)", "default": 20.0},
    },
    "Rate Pitch": {
        "ATC_RAT_PIT_P":    {"label": "P Gain",        "default": 0.135},
        "ATC_RAT_PIT_I":    {"label": "I Gain",        "default": 0.135},
        "ATC_RAT_PIT_D":    {"label": "D Gain",        "default": 0.0036},
        "ATC_RAT_PIT_IMAX": {"label": "I Max",         "default": 0.5},
        "ATC_RAT_PIT_FLTD": {"label": "D Filter (Hz)", "default": 20.0},
        "ATC_RAT_PIT_FLTT": {"label": "T Filter (Hz)", "default": 20.0},
    },
    "Rate Yaw": {
        "ATC_RAT_YAW_P":    {"label": "P Gain",        "default": 0.18},
        "ATC_RAT_YAW_I":    {"label": "I Gain",        "default": 0.018},
        "ATC_RAT_YAW_D":    {"label": "D Gain",        "default": 0.0},
        "ATC_RAT_YAW_IMAX": {"label": "I Max",         "default": 0.5},
        "ATC_RAT_YAW_FLTD": {"label": "D Filter (Hz)", "default": 2.5},
        "ATC_RAT_YAW_FLTE": {"label": "E Filter (Hz)", "default": 2.5},
    },
    "Angle": {
        "ATC_ANG_RLL_P":    {"label": "Roll P",        "default": 4.5},
        "ATC_ANG_PIT_P":    {"label": "Pitch P",       "default": 4.5},
        "ATC_ANG_YAW_P":    {"label": "Yaw P",         "default": 4.5},
    },
    "Position Z": {
        "PSC_ACCZ_P":       {"label": "AccZ P",        "default": 0.5},
        "PSC_ACCZ_I":       {"label": "AccZ I",        "default": 1.0},
        "PSC_ACCZ_D":       {"label": "AccZ D",        "default": 0.0},
        "PSC_VELZ_P":       {"label": "VelZ P",        "default": 5.0},
        "PSC_VELZ_I":       {"label": "VelZ I",        "default": 0.0},
        "PSC_VELZ_D":       {"label": "VelZ D",        "default": 0.0},
        "PSC_POSZ_P":       {"label": "PosZ P",        "default": 1.0},
    },
    "Position XY": {
        "PSC_VELXY_P":      {"label": "VelXY P",       "default": 2.0},
        "PSC_VELXY_I":      {"label": "VelXY I",       "default": 1.0},
        "PSC_VELXY_D":      {"label": "VelXY D",       "default": 0.5},
        "PSC_POSXY_P":      {"label": "PosXY P",       "default": 1.0},
    },
    "Motor": {
        "MOT_THST_HOVER":   {"label": "Hover Thrust",  "default": 0.35},
        "MOT_SPIN_MIN":     {"label": "Spin Min",      "default": 0.15},
        "MOT_SPIN_ARM":     {"label": "Spin Arm",      "default": 0.10},
        "MOT_PWM_MIN":      {"label": "PWM Min",       "default": 1000},
        "MOT_PWM_MAX":      {"label": "PWM Max",       "default": 2000},
    },
    "Optical Flow": {
        "FLOW_ENABLE":      {"label": "Enable",        "default": 1},
        "FLOW_FXSCALER":    {"label": "X Scaler",      "default": 0},
        "FLOW_FYSCALER":    {"label": "Y Scaler",      "default": 0},
        "EK3_FLOW_DELAY":   {"label": "EKF3 Delay",   "default": 10},
    },
}

# ===== Serial link =====
# Must match SERIAL2_BAUD on the flight controller (57 = 57600, 921 = 921600).
# Change BOTH sides or the link drops. At 57600 the usable inbound budget is
# ~5.7 kB/s, which is what caps the stream rates below.
SERIAL_BAUD = 57600

# ===== MAVLink stream rates =====
# ATTITUDE must exceed the fastest consumer loop — the 20 Hz correction loop in
# movement_controller. SR1_*/SR2_* are all 0 in mav.parm, so the runtime
# request in _configure_streams() is the only thing streaming it at all.
#
# EXTRA3 carries rangefinder (takeoff altitude, read at 10 Hz), vibration,
# battery, EKF status and OPTICAL_FLOW — so it is the stream to raise if the
# correction loop is ever moved onto flow velocity instead of attitude.
_HIGH_BAUD = SERIAL_BAUD >= 115200
ATTITUDE_RATE_HZ = 50 if _HIGH_BAUD else 30
EXTRA3_RATE_HZ   = 30 if _HIGH_BAUD else 10
POSITION_RATE_HZ = 10 if _HIGH_BAUD else 5

# ===== RC deadman windows (seconds) =====
# Socket clients heartbeat while a button is held, so the window can be tight.
# The HTTP path has no heartbeat, so it gets a longer backstop — enough for a
# manual curl test, short enough that a lost client cannot latch a command.
RC_DEADMAN_SOCKET_S = 0.5
RC_DEADMAN_HTTP_S   = 3.0

# ===== RC OverRide Config =====
DIR_MAP = {
    "forward":       ("pitch",    1350),
    "backward":      ("pitch",    1650),
    "right":         ("roll",     1650),
    "left":          ("roll",     1350),
    "yaw_right":     ("yaw",      1650),
    "yaw_left":      ("yaw",      1350),
    "throttle_up":   ("throttle", 1650),
    "throttle_down": ("throttle", 1350),
}


# ─── MAVLink stream rates ────────────────
def _configure_streams(veh):
    """
    Request per-stream MAVLink rates from the flight controller.

    DroneKit's connect(rate=N) sends a single MAV_DATA_STREAM_ALL request, i.e.
    "send me every stream at N Hz". At 57600 baud (~5.76 kB/s) that saturates
    the link well before 30 Hz — the FCU then drops and delays messages, and
    ATTITUDE arrives late along with everything else. Asking per-stream keeps
    the one rate that matters high and everything else cheap.

    What each stream actually carries on ArduCopter, and which DroneKit
    attribute it feeds:

      EXTRA1   ATTITUDE, AHRS2, PID_TUNING
               -> vehicle.attitude  (pitch/roll/yaw — the correction loop)
      EXTRA2   VFR_HUD
               -> vehicle.groundspeed, vehicle.airspeed, heading
      EXTRA3   RANGEFINDER, DISTANCE_SENSOR, OPTICAL_FLOW, VIBRATION,
               BATTERY_STATUS, EKF_STATUS_REPORT, AHRS, HWSTATUS, SYSTEM_TIME
               -> vehicle.rangefinder (takeoff altitude), vehicle.vibration,
                  vehicle.ekf_ok, battery detail. Heaviest stream by far.
      POSITION GLOBAL_POSITION_INT, LOCAL_POSITION_NED
               -> vehicle.location.*, vehicle.velocity
      EXT_STAT SYS_STATUS, POWER_STATUS, MEMINFO, MISSION_CURRENT,
               GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, FENCE_STATUS
               -> vehicle.battery (voltage/current), vehicle.gps_0
      RC_CHAN  RC_CHANNELS, SERVO_OUTPUT_RAW
               -> useful for verifying overrides land; cheap to keep low
      RAW_SENS RAW_IMU, SCALED_IMU2/3, SCALED_PRESSURE*
               -> unused here, and expensive. Off.

    vehicle.mode / vehicle.armed come from HEARTBEAT, which is always sent at
    1 Hz regardless of these settings.

    Rough inbound budget at 57600 (~5.7 kB/s usable):
        EXTRA1   30 Hz  ~1080 B/s
        EXTRA3   10 Hz  ~1200 B/s
        POSITION  5 Hz   ~350 B/s
        EXT_STAT  2 Hz   ~300 B/s
        EXTRA2    4 Hz   ~112 B/s
        RC_CHAN   2 Hz    ~80 B/s
        ------------------------------------
        total         ~3.1 kB/s   (~55% of link)

    The outbound RC override stream does not compete with this — serial is full
    duplex, so TX and RX have independent budgets.
    """
    streams = [
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,          ATTITUDE_RATE_HZ),
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,          EXTRA3_RATE_HZ),
        (mavutil.mavlink.MAV_DATA_STREAM_POSITION,        POSITION_RATE_HZ),
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,          4),
        (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),
        (mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,     2),
        (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,     0),
    ]
    for stream_id, rate in streams:
        try:
            veh._master.mav.request_data_stream_send(
                veh._master.target_system,
                veh._master.target_component,
                stream_id,
                int(rate),
                1 if rate > 0 else 0     # start / stop
            )
        except Exception as e:
            print(f"[stream] request id={stream_id} @{rate}Hz failed: {e}")
    print(f"[stream] configured @{SERIAL_BAUD} baud — "
          f"ATTITUDE {ATTITUDE_RATE_HZ}Hz, EXTRA3 {EXTRA3_RATE_HZ}Hz "
          f"(correction loop {CORR_RATE_HZ}Hz, RC out {RC_RATE_HZ}Hz)")


# ─── Waypoint Navigation Bridge ──────────
# WaypointNav holds no reference to the vehicle or the MovementController. It
# reads pose through one callable and writes sticks through another, which is
# what lets the identical control law be flown against the simulator in a
# browser with no Pixhawk attached.

def _nav_pose():
    """
    Current pose in the SAME frame the browser plot is drawn in: metres east,
    north and up of the arm point.

    Returns None when there is no usable reference frame — no aircraft, or no
    flight recording, which means no arm-point origin to measure from. Nav
    treats that as "cannot navigate" rather than guessing an origin, because a
    guessed origin puts every waypoint in the wrong place.
    """
    sim = tracker.sim_source() if tracker is not None else None
    if sim is not None and hasattr(sim, "pose"):
        return sim.pose()

    v = vehicle
    if v is None:
        return None

    origin = tracker.origin() if tracker is not None else (None, None, None)
    if origin[0] is None or origin[1] is None:
        return None

    try:
        lf = v.location.local_frame
    except Exception:
        lf = None
    if lf is None or lf.north is None or lf.east is None:
        return None

    down = lf.down if lf.down is not None else 0.0
    o_down = origin[2] if origin[2] is not None else 0.0

    def _get(fn, default=None):
        try:
            return fn()
        except Exception:
            return default

    return {
        "x": lf.east - origin[1],          # metres east of the arm point
        "y": lf.north - origin[0],         # metres north
        "z": -(down - o_down),             # metres up (NED down, flipped)
        "yaw": _get(lambda: v.attitude.yaw, 0.0),
        # LiveSource labels a sample "ekf" only when local_frame answered; this
        # path only exists because it did, so the label matches. If the EKF drops
        # out mid-flight local_frame goes None and this returns None above,
        # which aborts the leg rather than steering on stale numbers.
        "src": "ekf",
        "ekf_ok": bool(_get(lambda: v.ekf_ok, False)),
        "armed": bool(_get(lambda: v.armed, False)),
        "mode": _get(lambda: v.mode.name, None),
        "t": time.time(),
    }


def _nav_send(roll, pitch, throttle, yaw):
    """Apply stick values, and refresh the deadman so it does not centre them."""
    sim = tracker.sim_source() if tracker is not None else None
    if sim is not None and hasattr(sim, "apply_rc"):
        sim.apply_rc(roll, pitch, throttle, yaw)
        return
    if mc is None:
        raise RuntimeError("movement controller not initialised")
    mc.set_rc(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw)
    mc.touch_deadman(NAV_DEADMAN_S)


def _nav_param(name):
    v = vehicle
    if v is None:
        return None
    try:
        return v.parameters.get(name)
    except Exception:
        return None


def _nav_abort(reason):
    """
    Stop navigating. Called from every path that takes control away from nav —
    a manual stick command, hold, land, RTL, emergency, disarm, mode change.

    Silent no-op when nav is unavailable or idle, so callers do not have to
    guard, and wrapped so a nav fault can never break a flight command's
    response path.
    """
    if nav is None:
        return
    try:
        nav.abort(reason)
    except Exception as e:
        print(f"[nav] abort failed: {type(e).__name__}: {e}")


# nav.attach() happens further down, once _mark is defined.


# ─── Connect Drone ───────────────────────
def connect_drone():
    global vehicle, mc, mm
    print("Connecting to Pixhawk...")
    vehicle = connect(
        '/dev/ttyAMA0',
        baud=SERIAL_BAUD,
        wait_ready=False
    )
    print("Drone connected!")
    _configure_streams(vehicle)
    print("Speed params set: WPNAV_SPEED=30 UP=30 DN=30 cm/s")
    mc = MovementController(vehicle)
    # Share the controller — mm must drive the same instance whose RC sender
    # and correction threads are started below, otherwise mission commands go
    # into a dict nothing transmits.
    mm = MissionManager(vehicle, mc)
    print("MovementController and MissionManager initialised.")
    mc.start_rc_override()  # ← start on connect
    print("RC override thread started.")
    mc.start_correction_loop()  # ← start on connect for the correction loop

    # Flight tracker: watches the armed edge and records one flight per arm.
    if tracker is not None:
        try:
            tracker.attach(vehicle, mc, frame_hub, start_camera)
            tracker.start()
        except Exception as e:
            print(f"[tracker] failed to start: {type(e).__name__}: {e}")

# ─── Safe Value Helpers ──────────────────
# FIX 2: DroneKit fields can return None; these prevent TypeError crashes.

def safe_float(val, default=0.0):
    """Return float(val) or default if val is None / unconvertible."""
    try:
        return float(val) if val is not None else default
    except (TypeError, ValueError):
        return default

def safe_round(val, decimals=2, default=0.0):
    """Return round(float(val), decimals) or default if val is None."""
    try:
        return round(float(val), decimals) if val is not None else default
    except (TypeError, ValueError):
        return default

def safe_int(val, default=0):
    """Return int(val) or default if val is None / unconvertible."""
    try:
        return int(val) if val is not None else default
    except (TypeError, ValueError):
        return default

# ─── Helper Functions ────────────────────
def get_status_data():
    try:
        mode = vehicle.mode.name if vehicle.mode else "UNKNOWN"
    except Exception:
        mode = "UNKNOWN"

    try:
        armed = bool(vehicle.armed)
    except Exception:
        armed = False

    try:
        rf = safe_float(vehicle.rangefinder.distance)
        rangefinder_dist = round(rf if rf > 0 else 0.0, 2)
    except Exception:
        rangefinder_dist = 0.0

    try:
        altitude = safe_round(vehicle.location.global_relative_frame.alt)
    except Exception:
        altitude = 0.0

    try:
        battery_v = safe_round(vehicle.battery.voltage, 1)
    except Exception:
        battery_v = 0.0

    try:
        battery_lvl = vehicle.battery.level  # int or None
    except Exception:
        battery_lvl = None

    try:
        satellites = safe_int(vehicle.gps_0.satellites_visible)
        gps_fix    = safe_int(vehicle.gps_0.fix_type)
    except Exception:
        satellites = 0
        gps_fix    = 0

    try:
        pitch = safe_round(vehicle.attitude.pitch, 6)
        roll  = safe_round(vehicle.attitude.roll,  6)
        yaw   = safe_round(vehicle.attitude.yaw,   6)
    except Exception:
        pitch = roll = yaw = 0.0

    try:
        lat = vehicle.location.global_frame.lat
        lon = vehicle.location.global_frame.lon
    except Exception:
        lat = lon = 0.0

    try:
        groundspeed = safe_round(vehicle.groundspeed, 1)
        airspeed    = safe_round(vehicle.airspeed,    1)
    except Exception:
        groundspeed = airspeed = 0.0

    try:
        ekf_ok     = bool(vehicle.ekf_ok)
        is_armable = bool(vehicle.is_armable)
    except Exception:
        ekf_ok = is_armable = False

    try:
        vibe_x = safe_round(vehicle.vibration.vibration_x, 3)
        vibe_y = safe_round(vehicle.vibration.vibration_y, 3)
        vibe_z = safe_round(vehicle.vibration.vibration_z, 3)
    except Exception:
        vibe_x = vibe_y = vibe_z = 0.0

    return {
        "mode":          mode,
        "armed":         armed,
        "rangefinder":   rangefinder_dist,
        "altitude":      altitude,
        "battery":       battery_v,
        "battery_level": battery_lvl,
        "satellites":    satellites,
        "gps_fix":       gps_fix,
        "pitch":         pitch,
        "roll":          roll,
        "yaw":           yaw,
        "lat":           lat,
        "lon":           lon,
        "groundspeed":   groundspeed,
        "airspeed":      airspeed,
        "ekf_ok":        ekf_ok,
        "is_armable":    is_armable,
        "vibe_x":        vibe_x,
        "vibe_y":        vibe_y,
        "vibe_z":        vibe_z,
    }

# send_stop(), send_yaw() and _send_yaw_locked() were removed on 2026-08-07.
# They were module-level GUIDED-mode senders (BODY_NED zero-velocity and
# MAV_CMD_CONDITION_YAW) with no callers anywhere — duplicates of the
# MovementController methods deleted in the same pass.

# ─── Telemetry Stream ────────────────────
def stream_telemetry():
    while True:
        try:
            if vehicle:
                socketio.emit('telemetry', get_status_data())
        except Exception as e:
            print(f"Telemetry error: {e}")
        time.sleep(1.0)

# ─── Camera Functions ────────────────────
def start_camera():
    global camera, camera_active

    print("camera started")

    # Early exit if already running (checked outside lock for speed)
    with camera_lock:
        if camera is not None:
            return

    print("Video Capture is starting")

    # Open and configure outside lock so 1s settle sleep doesn't block other threads
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    time.sleep(1)  # let V4L2 driver settle

    with camera_lock:
        if camera is None:  # re-check: another thread may have started it during sleep
            if cap.isOpened():
                ret, frame = cap.read()

                if not ret:
                    cap.release()
                    raise RuntimeError("camera opened but failed to read first frame")

                camera        = cap
                camera_active = True
                print("Camera started")
                if frame_hub is not None and not frame_hub.running:
                    frame_hub.start()
                    print("[framehub] grabber started")
            else:
                cap.release()
                print("Failed to start camera")

def stop_camera():
    global camera, camera_active

    # A flight video recording is reading through the hub. Releasing the
    # capture underneath it would truncate the recording, so refuse.
    if tracker is not None and tracker.status().get("video_active"):
        print("Camera stop refused: flight video recording in progress")
        return False

    if frame_hub is not None and frame_hub.running:
        frame_hub.stop()

    with camera_lock:
        if camera is not None:
            camera.release()
            camera      = None
            camera_active = False
            print("Camera stopped")
    return True

def generate_frames():
    print("[Cam] Generator Started")
    start_camera()

    error_count = 0
    max_errors  = 10  # stop stream after 10 consecutive read failures
    seq         = 0   # last frame sequence this viewer has sent

    while camera_active:

        if frame_hub is not None:
            # Hub path: one grabber thread owns cap.read(), viewers take the
            # latest frame. Two browser tabs no longer halve each other's rate,
            # and the flight video recorder can share the same frames.
            seq, frame, _ = frame_hub.wait_for_new(seq, timeout=2.0)
            success = frame is not None
        else:
            # Fallback when the tracker package is unavailable — original
            # behaviour, one reader straight off the capture.
            with camera_lock:
                cap = camera
            if cap is None:
                break
            success, frame = cap.read()

        if not success:
            error_count += 1
            if error_count >= max_errors:
                print(f"Camera: {max_errors} consecutive failures, stopping stream")
                break
            socketio.sleep(0.1)
            continue

        error_count = 0

        ret, buffer = cv2.imencode(
            '.jpg', frame,
            [cv2.IMWRITE_JPEG_QUALITY, CAMERA_JPEG_QUALITY]
        )
        if not ret:
            continue

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            buffer.tobytes() +
            b'\r\n'
        )

        if frame_hub is None:
            socketio.sleep(1.0 / CAMERA_FPS)  # pace to actual camera FPS
        else:
            # wait_for_new() already blocks until the grabber produces a frame,
            # so the hub sets the pace. Sleeping again here would skip every
            # second frame and halve the stream rate.
            socketio.sleep(0)

# ─── Guard Helpers ───────────────────────
def _no_vehicle():
    return jsonify({"success": False, "message": "Drone not connected"}), 503


def _mark(kind, detail=None):
    """
    Drop a marker on the recording flight's timeline. No-op when nothing is
    recording. Wrapped so a tracker fault can never propagate into a flight
    command's response path.
    """
    if tracker is None:
        return
    try:
        tracker.mark(kind, detail)
    except Exception:
        pass


# Deferred until here because attach() captures _mark, which is defined just
# above. The loop is started now rather than in connect_drone() so waypoint
# navigation can be flown against the simulator with no Pixhawk attached.
if nav is not None:
    nav.attach(pose_fn=_nav_pose, send_fn=_nav_send,
               mark_fn=_mark, param_fn=_nav_param)
    nav.start()


# ─── Routes ──────────────────────────────

@app.route('/')
def index():
    return render_template('index.html')

# FIX 3: health endpoint
@app.route('/health', methods=['GET'])
def health():
    connected = vehicle is not None
    if not connected:
        return jsonify({
            "connected":     False,
            "armed":         False,
            "mode":          None,
            "battery":       None,
            "gps_fix":       None,
            "system_status": "DISCONNECTED",
        })
    try:
        sys_status = vehicle.system_status.state if vehicle.system_status else "UNKNOWN"
    except Exception:
        sys_status = "UNKNOWN"

    try:
        return jsonify({
            "connected":     True,
            "armed":         bool(vehicle.armed),
            "mode":          vehicle.mode.name if vehicle.mode else "UNKNOWN",
            "battery":       safe_round(vehicle.battery.voltage, 1) if vehicle.battery else 0.0,
            "gps_fix":       safe_int(vehicle.gps_0.fix_type) if vehicle.gps_0 else 0,
            "system_status": sys_status,
            "tracker":       tracker.status() if tracker else {"available": False},
            "nav":           nav.status() if nav else {"available": False, "error": NAV_ERROR},
        })
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/lvlcal', methods=['POST'])
def lvlcal():
    if not vehicle:
        return _no_vehicle()
    try:
        msg = vehicle.message_factory.command_long_encode(
            1, 1,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0,
            0, 0, 0, 0,
            2,
            0, 0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        return jsonify({"success": True, "message": "Level Calibrated"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/status', methods=['GET'])
def status():
    if not vehicle:
        return jsonify({"success": False, "message": "Not connected"}), 500
    try:
        return jsonify(get_status_data())
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

def _arm_drone():
    try:
        mc._switch_mode("LOITER")
        mc._arm()
        return jsonify({"success": True, "message": "inside_arm_drone"})
    finally:
        _cmd_lock.release()

@app.route('/arm', methods=['POST'])
def arm():
    if not vehicle:
        return _no_vehicle()
    if not _cmd_lock.acquire(blocking=False):
        return jsonify({"success": False, "message": "Another command is in progress"}), 409
    try:
        if vehicle.armed:
            _cmd_lock.release()
            return jsonify({"success": False, "message": "Already armed"})
        thread = threading.Thread(target=_arm_drone)
        thread.daemon = True
        thread.start()
        return jsonify({"success": True, "message": "Arming..."})
    except Exception as e:
        _cmd_lock.release()
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/disarm', methods=['POST'])
def disarm():
    if not vehicle:
        return _no_vehicle()
    try:
        if not vehicle.armed:
            return jsonify({"success": False, "message": "Already disarmed"})

        _nav_abort("disarm")
        rf = safe_float(vehicle.rangefinder.distance)
        if rf <= 0:
            return jsonify({
                "success": False,
                "message": "Rangefinder reading invalid (0.0) — cannot verify altitude. Land manually."
            }), 400
        if rf > 0.17:
            return jsonify({
                "success": False,
                "message": f"Too high to disarm ({rf:.2f}m). Land first (need <0.17m)."
            }), 400

        # Force-disarm: param2=21196 bypasses ArduCopter's "motors running" safety check.
        msg = vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,      # 0 = disarm
            21196,  # force magic number
            0, 0, 0, 0, 0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(0.5)
        return jsonify({"success": True, "message": f"Disarmed (rangefinder: {rf:.2f}m)"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/safety', methods=['POST'])
def safety():
    if not vehicle:
        return _no_vehicle()
    try:
        data  = request.get_json() or {}
        state = data.get('state', 'on').lower()

        if state == 'off':
            msg = vehicle.message_factory.command_long_encode(
                1, 1,
                mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            vehicle.send_mavlink(msg)
            vehicle.flush()
            return jsonify({"success": True, "message": "Safety switch DISABLED (Ready to Arm)"})

        elif state == 'on':
            msg = vehicle.message_factory.command_long_encode(
                1, 1,
                mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            vehicle.send_mavlink(msg)
            vehicle.flush()
            return jsonify({"success": True, "message": "Safety switch ENABLED (Locked)"})

        else:
            return jsonify({"success": False, "message": "Invalid state. Use 'on' or 'off'."}), 400
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/takeoff', methods=['POST'])
def takeoff():
    if not vehicle:
        return _no_vehicle()
    if not _cmd_lock.acquire(blocking=False):
        return jsonify({"success": False, "message": "Another command is in progress"}), 409

    try:
        data     = request.json or {}
        altitude = float(data.get('altitude', 2))

        if safe_float(vehicle.battery.voltage) < 13.2:
            _cmd_lock.release()
            return jsonify({"success": False, "message": "Battery too low (4S min 13.2v)"}), 400

        if altitude > 10:
            _cmd_lock.release()
            return jsonify({"success": False, "message": "Max altitude is 10m"}), 400

        if altitude < 0.5:
            _cmd_lock.release()
            return jsonify({"success": False, "message": "Min altitude is 0.5m"}), 400

        _nav_abort("takeoff")
        _mark("takeoff", f"target {altitude}m")

        def _takeoff_worker():
            try:
                res = mc.takeoff(altitude)
                _mark("takeoff_done", (res or {}).get("message"))
            finally:
                _cmd_lock.release()

        thread = threading.Thread(target=_takeoff_worker, daemon=True)
        thread.start()

        return jsonify({
            "success":  True,
            "message":  f"Taking off to {altitude}m",
            "altitude": altitude,
        })
    except Exception as e:
        _cmd_lock.release()
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/land', methods=['POST'])
def land():
    global _takeoff_cancelled
    if not vehicle:
        return _no_vehicle()
    try:
        _takeoff_cancelled = True
        _nav_abort("land")
        vehicle.mode = VehicleMode("LAND")
        _mark("land")
        return jsonify({"success": True, "message": "Landing"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500
        

@app.route('/rtl', methods=['POST'])
def rtl():
    global _takeoff_cancelled
    if not vehicle:
        return _no_vehicle()
    try:
        if safe_int(vehicle.gps_0.fix_type) < 3:
            return jsonify({"success": False, "message": "No GPS fix for RTL"}), 400
        _takeoff_cancelled = True
        _nav_abort("rtl")
        vehicle.mode = VehicleMode("RTL")
        _mark("rtl")
        return jsonify({"success": True, "message": "Returning to launch"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

# /move and /yaw were removed on 2026-08-07 along with the GUIDED-mode
# primitives they called. Both are answered explicitly rather than 404'd so an
# old client (or an LLM working from stale docs) gets a usable pointer.
@app.route('/move', methods=['POST'])
@app.route('/yaw', methods=['POST'])
def _removed_guided_motion():
    return jsonify({
        "success": False,
        "message": ("Removed — these used GUIDED-mode setpoints, which this "
                    "airframe does not fly. Use POST /rc or the 'rc' socket "
                    "event with a direction from DIR_MAP."),
        "use_instead": "/rc",
        "directions": sorted(DIR_MAP.keys()),
    }), 410

@app.route('/mode', methods=['POST'])
def set_mode():
    if not vehicle:
        return _no_vehicle()
    try:
        data = request.json or {}
        mode = data.get('mode', 'STABILIZE').upper()

        allowed_modes = [
            'STABILIZE', 'ALTHOLD', 'LOITER',
            'GUIDED', 'GUIDED_NOGPS', 'LAND',
            'RTL', 'AUTO', 'POSHOLD',
        ]

        if mode not in allowed_modes:
            return jsonify({"success": False, "message": f"Mode must be one of {allowed_modes}"}), 400

        _nav_abort(f"mode change to {mode}")
        vehicle.mode = VehicleMode(mode)
        time.sleep(0.5)
        return jsonify({"success": True, "message": f"Mode changed to {mode}", "mode": mode})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/param', methods=['GET'])
def get_param():
    if not vehicle:
        return _no_vehicle()
    name = request.args.get('name')
    if not name:
        return jsonify({"success": False, "message": "Param name required"}), 400
    try:
        value = vehicle.parameters[name]
        return jsonify({"param": name, "value": value})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 404

@app.route('/param', methods=['POST'])
def set_param():
    if not vehicle:
        return _no_vehicle()
    try:
        data  = request.json or {}
        param = data.get('param')
        value = data.get('value')

        if not param or value is None:
            return jsonify({"success": False, "message": "param and value required"}), 400

        vehicle.parameters[param] = value
        time.sleep(0.2)
        return jsonify({"success": True, "param": param, "value": value})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/hold', methods=['POST'])
def hold():
    global _takeoff_cancelled, _motion_cancelled
    if not vehicle:
        return _no_vehicle()
    try:
        _takeoff_cancelled = True
        _motion_cancelled  = True
        _nav_abort("hold")
        res = mc.hold()
        if res["ok"]:
            return jsonify({"success": True, "message": res["message"]})
        return jsonify({"success": False, "message": res["message"]}), 500
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/emergency', methods=['POST'])
def emergency():
    global _takeoff_cancelled, _motion_cancelled
    if not vehicle:
        return _no_vehicle()
    # Emergency bypasses command lock intentionally
    try:
        _takeoff_cancelled = True
        _motion_cancelled  = True
        _nav_abort("emergency")
        mc.emergency_stop()
        _mark("emergency")
        return jsonify({"success": True, "message": "Emergency stop executed"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

# ─── Mission Endpoints ───────────────────

@app.route('/mission', methods=['POST'])
def mission_start():
    if not mm:
        return jsonify({"success": False, "message": "Drone not connected"}), 503
    if mm.is_busy():
        return jsonify({"success": False, "message": "Mission already running"}), 409
    try:
        data  = request.json or {}
        steps = data.get('steps', [])
        if not steps:
            return jsonify({"success": False, "message": "steps list required"}), 400
        _nav_abort("mission started")
        mm.run_mission(steps, blocking=False)
        _mark("mission", ",".join(str(s.get("cmd")) for s in steps))
        return jsonify({"success": True, "message": "Mission started", "steps": len(steps)})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/mission/status', methods=['GET'])
def mission_status():
    return jsonify(mm.status if mm else {})

@app.route('/mission/cancel', methods=['POST'])
def mission_cancel():
    if not mm:
        return jsonify({"success": False, "message": "Drone not connected"}), 503
    mm.mc.cancel()
    return jsonify({"success": True, "message": "Mission cancel signal sent"})

# ─── Camera Routes ───────────────────────

@app.route('/camera/stream', methods=['GET'])
def video_feed():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/camera/start', methods=['GET', 'POST'])
def camera_on():
    try:
        start_camera()
        return jsonify({"success": True, "message": "Camera started"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/camera/stop', methods=['GET', 'POST'])
def camera_off():
    try:
        if not stop_camera():
            return jsonify({
                "success": False,
                "message": "Flight video recording in progress — camera kept on"
            }), 409
        return jsonify({"success": True, "message": "Camera stopped"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

# ─── Page Routes ─────────────────────────

# @app.route('/tuner')
# def tuner():
#     return render_template('pid_tuner.html')

@app.route('/network')
def network_page():
    return render_template('network.html')

# ─── Network aliases (frontend uses /network/*, server has /wifi/*) ──
@app.route('/network/scan', methods=['GET'])
def network_scan():
    try:
        result = subprocess.run(
            ['/usr/bin/nmcli', '-t', '-f', 'SSID,SIGNAL,SECURITY', 'device', 'wifi', 'list'],
            capture_output=True, text=True, timeout=10
        )
        networks = []
        for line in result.stdout.strip().split('\n'):
            parts = line.split(':')
            if len(parts) >= 2 and parts[0]:
                networks.append({
                    "ssid":     parts[0],
                    "signal":   parts[1] if len(parts) > 1 else '',
                    "security": parts[2] if len(parts) > 2 else '',
                })
        return jsonify({"success": True, "networks": networks})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/network/connect', methods=['POST'])
def network_connect():
    try:
        data     = request.json or {}
        ssid     = data.get('ssid')
        password = data.get('password', '')
        if not ssid:
            return jsonify({"success": False, "message": "ssid required"}), 400
        cmd = ['sudo', '/usr/bin/nmcli', 'device', 'wifi', 'connect', ssid]
        if password:
            cmd += ['password', password]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        if result.returncode == 0:
            return jsonify({"success": True, "message": f"Connected to {ssid}"})
        return jsonify({"success": False, "message": result.stderr.strip()}), 500
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

# ─── PID Tuner API ───────────────────────

@app.route('/pid/all', methods=['GET'])
def pid_all():
    if not vehicle:
        return _no_vehicle()
    result = {}
    for group, params in PID_GROUPS.items():
        result[group] = {}
        for param, meta in params.items():
            try:
                val = vehicle.parameters[param]
                val = round(float(val), 6) if val is not None else None
            except Exception:
                val = None
            result[group][param] = {"label": meta["label"], "value": val}
    return jsonify(result)

@app.route('/pid/set', methods=['POST'])
def pid_set():
    if not vehicle:
        return _no_vehicle()
    try:
        data  = request.json or {}
        param = data.get('param')
        value = data.get('value')
        if not param or value is None:
            return jsonify({"success": False, "error": "param and value required"}), 400
        vehicle.parameters[param] = float(value)
        time.sleep(0.2)
        return jsonify({"success": True, "param": param, "value": value})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route('/pid/reset', methods=['POST'])
def pid_reset():
    if not vehicle:
        return _no_vehicle()
    try:
        data  = request.json or {}
        param = data.get('param')
        if not param:
            return jsonify({"success": False, "error": "param required"}), 400
        default_val = None
        for group_params in PID_GROUPS.values():
            if param in group_params:
                default_val = group_params[param]["default"]
                break
        if default_val is None:
            return jsonify({"success": False, "error": f"No default for {param}"}), 400
        vehicle.parameters[param] = float(default_val)
        time.sleep(0.2)
        return jsonify({"success": True, "param": param, "default_value": default_val})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route('/pid/save', methods=['POST'])
def pid_save():
    if not vehicle:
        return _no_vehicle()
    try:
        data     = request.json or {}
        filename = data.get('filename', 'pid_backup.json')
        if not filename.endswith('.json'):
            filename += '.json'
        os.makedirs(PID_SAVES_DIR, exist_ok=True)
        saved = {}
        for group, params in PID_GROUPS.items():
            for param in params:
                try:
                    val = vehicle.parameters[param]
                    if val is not None:
                        saved[param] = round(float(val), 6)
                except Exception:
                    pass
        path = os.path.join(PID_SAVES_DIR, filename)
        with open(path, 'w') as f:
            json.dump(saved, f, indent=2)
        return jsonify({"success": True, "filename": filename, "count": len(saved)})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route('/pid/files', methods=['GET'])
def pid_files():
    try:
        os.makedirs(PID_SAVES_DIR, exist_ok=True)
        files = [f for f in os.listdir(PID_SAVES_DIR) if f.endswith('.json')]
        return jsonify({"files": sorted(files)})
    except Exception as e:
        return jsonify({"files": [], "error": str(e)})

@app.route('/pid/load', methods=['POST'])
def pid_load():
    if not vehicle:
        return _no_vehicle()
    try:
        data     = request.json or {}
        filename = data.get('filename')
        if not filename:
            return jsonify({"success": False, "error": "filename required"}), 400
        path = os.path.join(PID_SAVES_DIR, filename)
        if not os.path.exists(path):
            return jsonify({"success": False, "error": f"{filename} not found"}), 404
        with open(path) as f:
            params = json.load(f)
        loaded = 0
        for param, val in params.items():
            try:
                vehicle.parameters[param] = float(val)
                loaded += 1
                time.sleep(0.05)
            except Exception:
                pass
        return jsonify({"success": True, "loaded": loaded, "filename": filename})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

# ─── WiFi Management ─────────────────────

@app.route('/wifi/scan', methods=['GET'])
def wifi_scan():
    try:
        import subprocess
        result = subprocess.run(
            ['/usr/bin/nmcli', '-t', '-f', 'SSID,SIGNAL,SECURITY', 'device', 'wifi', 'list'],
            capture_output=True, text=True, timeout=10
        )
        networks = []
        for line in result.stdout.strip().split('\n'):
            parts = line.split(':')
            if len(parts) >= 2 and parts[0]:
                networks.append({
                    "ssid":     parts[0],
                    "signal":   parts[1] if len(parts) > 1 else '',
                    "security": parts[2] if len(parts) > 2 else '',
                })
        return jsonify({"success": True, "networks": networks})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/wifi/connect', methods=['POST'])
def wifi_connect():
    try:
        import subprocess
        data     = request.json or {}
        ssid     = data.get('ssid')
        password = data.get('password', '')
        if not ssid:
            return jsonify({"success": False, "message": "ssid required"}), 400
        cmd = ['sudo', '/usr/bin/nmcli', 'device', 'wifi', 'connect', ssid]
        if password:
            cmd += ['password', password]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        if result.returncode == 0:
            return jsonify({"success": True, "message": f"Connected to {ssid}"})
        return jsonify({"success": False, "message": result.stderr.strip()}), 500
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

# ─── Command Queue Endpoints (FIX 6) ─────
# Skeleton only. A future mission sequencer will pop from _cmd_queue
# and dispatch commands sequentially. Currently queue is write-only.

@app.route('/queue/add', methods=['POST'])
def queue_add():
    """Add a command to the mission queue (not yet auto-executed)."""
    try:
        data   = request.json or {}
        action = data.get('action')
        if not action:
            return jsonify({"success": False, "message": "action required"}), 400
        item = {"action": action, "params": data.get('params', {})}
        with _cmd_queue_lock:
            _cmd_queue.append(item)
            depth = len(_cmd_queue)
        return jsonify({"success": True, "queued": depth, "item": item})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/queue/status', methods=['GET'])
def queue_status():
    with _cmd_queue_lock:
        items = list(_cmd_queue)
    return jsonify({"queued": len(items), "items": items})

@app.route('/queue/clear', methods=['POST'])
def queue_clear():
    with _cmd_queue_lock:
        _cmd_queue.clear()
    return jsonify({"success": True, "message": "Queue cleared"})


# RC control endpoint

def _rc_apply(data, deadman_s):
    """
    Shared RC handler for both the HTTP endpoint and the Socket.IO event.

    Returns (payload_dict, http_status). Socket callers ignore the status.

    deadman_s is how long a non-neutral command stays valid without a refresh.
    Every 'move' refreshes it; when it expires, movement_controller centres all
    channels. Pass 0 to disable (bench testing only — a dropped client will then
    leave the command latched, which is the pre-2026-08-07 behaviour).
    """
    if not vehicle or mc is None:
        return {"success": False, "message": "Drone not connected"}, 503

    action = data.get("action", "move")

    # Manual input always wins. Without this the operator's stick command and
    # the nav loop would both write the same four channels ten times a second
    # and the aircraft would follow whichever wrote last — which is neither.
    # Abort first, then apply, so nav cannot overwrite the command that just
    # took control away from it.
    if action in ("move", "hold"):
        _nav_abort("manual RC input")

    if action == "start":
        mc.start_rc_override()
        return {"success": True, "message": "RC started"}, 200

    if action == "stop":
        mc.stop_rc_override()
        return {"success": True, "message": "RC stopped"}, 200

    if action == "hold":
        mc.rc_hold()
        return {"success": True, "message": "Holding"}, 200

    if action == "move":
        direction = data.get("direction")
        release   = data.get("release", False)

        if direction not in DIR_MAP:
            return {"success": False,
                    "message": f"Unknown: {direction}"}, 400

        channel, default = DIR_MAP[direction]

        if release:
            mc.rc_hold(channel)
            return {"success": True,
                    "message": f"{direction} released"}, 200

        value = int(data.get("value", default))
        mc.set_rc(**{channel: value})
        # Arm/refresh the deadman AFTER the command lands.
        mc.touch_deadman(data.get("deadman", deadman_s))
        return {"success": True,
                "message": f"{direction}={value}"}, 200

    return {"success": False, "message": "Unknown action"}, 400


@app.route('/rc', methods=['POST'])
def rc():
    payload, status = _rc_apply(request.json or {}, RC_DEADMAN_HTTP_S)
    return jsonify(payload), status

# WebRTC endpoint



# ─── WebSocket ───────────────────────────

@socketio.on('connect')
def on_connect():
    global _telemetry_started
    print("Client connected!")
    # FIX 1: start telemetry thread exactly once across all clients
    with _telemetry_lock:
        if not _telemetry_started:
            _telemetry_started = True
            t = threading.Thread(target=stream_telemetry)
            t.daemon = True
            t.start()

    # Tell the new client whether a flight is already recording, so a page
    # opened mid-flight can attach to the live track instead of waiting for
    # the next 'flight_started'.
    if tracker is not None:
        try:
            socketio.emit('tracker_status', tracker.status())
        except Exception:
            pass

    # Same reasoning for navigation: a page opened while the aircraft is already
    # flying a waypoint needs to render the pending route, not an empty panel.
    if nav is not None:
        try:
            socketio.emit('nav_status', nav.status())
        except Exception:
            pass

@socketio.on('disconnect')
def on_disconnect():
    print("Client disconnected!")
    # A client that vanishes mid-hold must not leave a stick latched. The
    # deadman would catch this within RC_DEADMAN_SOCKET_S anyway; centring here
    # makes it immediate.
    try:
        if mc is not None:
            mc.rc_hold()
            print("[rc] client gone — channels centred")
    except Exception as e:
        print(f"[rc] centre-on-disconnect failed: {e}")


# ─── RC over Socket.IO ───────────────────────────────────────────
# Same payload shape as POST /rc, but over the already-open websocket:
# no per-command TCP/TLS setup, no ngrok HTTP round trip. Roughly 100ms
# faster per input. POST /rc still works and is unchanged.
#
# Clients MUST re-emit 'rc' with the same direction every ~200ms while a
# button is held. That heartbeat is what keeps the deadman fed.
@socketio.on('rc')
def on_rc(data):
    payload, _ = _rc_apply(data or {}, RC_DEADMAN_SOCKET_S)
    return payload          # delivered to the client's ack callback

# ─── Main ────────────────────────────────
if __name__ == '__main__':

    WIFI_SSID = "dynamics"
    WIFI_PASS = "123456789"

    os.makedirs(PID_SAVES_DIR, exist_ok=True)

    scan_networks()
    if not connect_wifi(ssid=WIFI_SSID, password=WIFI_PASS):
        wifi_connection_retries(ssid=WIFI_SSID, password=WIFI_PASS)

    monitor = threading.Thread(target=wifi_monitor_loop, args=(WIFI_SSID, WIFI_PASS))
    monitor.daemon = True
    monitor.start()
    print(f"[wifi_monitor] Background monitor started for '{WIFI_SSID}'")

    time.sleep(5)

    ngrok_process = start_ngrok()

    #url = get_ngrok_url()
    #if url:
#	print(f"Public URL: {url}")

    drone_thread = threading.Thread(target=connect_drone)
    drone_thread.daemon = True
    drone_thread.start()
    print("API Server running on port 5000")
    print("\nEndpoints:")
    print("GET  /health")
    print("POST /lvlcal         calibrates level")
    print("GET  /status")
    print("POST /arm")
    print("POST /disarm")
    print("POST /safety         {state: on/off}")
    print("POST /takeoff        {altitude: 2}")
    print("POST /land")
    print("POST /rtl")
    print("POST /hold")
    print("POST /mode           {mode}")
    print("GET  /param?name=X")
    print("POST /param          {param, value}")
    print("POST /emergency")
    print("")
    print("RC (the control path):")
    print("POST /rc             {action: start|stop|hold|move, direction, release}")
    print("  socket 'rc'        same payload, ~100ms faster — preferred")
    print(f"  directions:        {', '.join(sorted(DIR_MAP.keys()))}")
    print(f"  deadman:           socket {RC_DEADMAN_SOCKET_S}s / http {RC_DEADMAN_HTTP_S}s")
    print("")
    print("POST /mission        {steps:[{cmd:takeoff|land|hold|hover, ...}]}")
    print("GET  /mission/status")
    print("POST /mission/cancel")
    print("GET  /camera/stream  (MJPEG)")
    print("POST /camera/start")
    print("POST /camera/stop")
    print("GET  /network/scan")
    print("POST /network/connect {ssid, password}")
    print("POST /queue/add      {action, params}   (not drained — no-op)")
    print("GET  /queue/status")
    print("POST /queue/clear")
    print("")
    if TRACKER_AVAILABLE:
        print(f"Flight tracker ({TRACK_HZ:g} Hz, one flight per arm):")
        print("GET  /flights              list past flights")
        print("GET  /flights/current      live recording status")
        print("GET  /flights/<id>         metadata + summary + events")
        print("GET  /flights/<id>/track   ?fields=x_m,y_m,z_m&decimate=N")
        print("GET  /flights/<id>/events  timeline markers")
        print("GET  /flights/<id>/video   mp4, HTTP Range (seekable)")
        print("POST /flights/<id>/label   {label, notes}")
        print("POST /flights/<id>/mark    {detail}  marker on live timeline")
        print("DEL  /flights/<id>         ?keep_video=1")
        print("GET  /flights/stats")
        print("POST /flights/sim/start    {profile:'square'|'nav'} no Pixhawk needed")
        print("POST /flights/sim/stop")
        print("  socket: flight_started / track_point / flight_ended")
        print(f"  db:     {os.path.abspath(FLIGHTS_DB)}")
        print(f"  video:  {os.path.abspath(FLIGHTS_VIDEO)}")
    else:
        print(f"Flight tracker DISABLED — {TRACKER_ERROR}")
    print("")
    if nav is not None:
        print("Click-to-waypoint navigation:")
        print("POST /nav/goto    {x, y, z?, replace?}  metres east/north/up of arm point")
        print("POST /nav/queue   {points:[{x,y,z?}, ...]}")
        print("GET  /nav/status")
        print("POST /nav/abort   {reason?}")
        print("POST /nav/clear")
        print("  socket: nav_status / nav_reached / nav_aborted")
        print(f"  limits: {NAV_MAX_RADIUS:.0f}m radius, LOITER + EKF fix required")
        print("  aborts on: manual RC, hold, land, rtl, mode change, disarm, emergency")
    else:
        print(f"Waypoint navigation DISABLED — {NAV_ERROR}")
    print("")
    print("GONE: /move, /yaw  -> 410, use /rc (GUIDED-mode, removed)")

    # Start grabbing before serving. The pre-roll buffer only has anything in it
    # if the camera was already running when the aircraft armed, so waiting for
    # the first browser to open the stream would make pre-roll depend on someone
    # happening to be watching.
    if CAMERA_AUTOSTART:
        try:
            start_camera()
            print(f"[camera] autostarted  {CAMERA_FPS} fps  "
                  f"pre-roll {CAMERA_PREROLL_S:g}s "
                  f"({int(CAMERA_PREROLL_S * CAMERA_FPS)} frames buffered)")
        except Exception as e:
            # Not fatal: no camera must never stop the flight server starting.
            print(f"[camera] autostart failed — {type(e).__name__}: {e}")

    socketio.run(
        app,
        host='0.0.0.0',
        port=5000,
        debug=False,
        allow_unsafe_werkzeug=True
    )
