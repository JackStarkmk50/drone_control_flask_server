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

from movement_controller import MovementController
from mission_manager import MissionManager

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

app = Flask(__name__,
            template_folder='../templates',
            static_folder='../static')

CORS(app)  # handles OPTIONS preflight responses

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

# ─── Connect Drone ───────────────────────
def connect_drone():
    global vehicle, mc, mm
    print("Connecting to Pixhawk...")
    vehicle = connect(
        '/dev/ttyAMA0',
        baud=57600,
        wait_ready=False
    )
    print("Drone connected!")
    mc = MovementController(vehicle)
    mm = MissionManager(vehicle)
    print("MovementController and MissionManager initialised.")

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

def send_stop():
    msg = vehicle.message_factory\
        .set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)

def send_yaw(heading, speed, direction, relative):
    msg = vehicle.message_factory\
        .command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,   # degrees
        speed,     # speed deg/s
        direction, # 1=CW -1=CCW
        relative,  # 1=relative 0=absolute
        0, 0, 0
    )
    vehicle.send_mavlink(msg)

def _send_yaw_locked(heading, speed, direction, relative):
    """Wrapper that releases _cmd_lock after send_yaw completes."""
    try:
        send_yaw(heading, speed, direction, relative)
    finally:
        _cmd_lock.release()

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

    # Early exit if already running (checked outside lock for speed)
    with camera_lock:
        if camera is not None:
            return

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
                camera        = cap
                camera_active = True
                print("Camera started")
            else:
                cap.release()
                print("Failed to start camera")

def stop_camera():
    global camera, camera_active

    with camera_lock:
        if camera is not None:
            camera.release()
            camera      = None
            camera_active = False
            print("Camera stopped")

def generate_frames():
    start_camera()

    error_count = 0
    max_errors  = 10  # stop stream after 10 consecutive read failures

    while camera_active:
        # Grab local reference under lock — prevents crash if stop_camera() runs concurrently
        with camera_lock:
            cap = camera

        if cap is None:
            break

        success, frame = cap.read()

        if not success:
            error_count += 1
            if error_count >= max_errors:
                print(f"Camera: {max_errors} consecutive failures, stopping stream")
                stop_camera()
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

        socketio.sleep(1.0 / CAMERA_FPS)  # pace to actual camera FPS

# ─── Guard Helpers ───────────────────────
def _no_vehicle():
    return jsonify({"success": False, "message": "Drone not connected"}), 503

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

        def _takeoff_worker():
            try:
                mc.takeoff(altitude)
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
        vehicle.mode = VehicleMode("LAND")
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
        vehicle.mode = VehicleMode("RTL")
        return jsonify({"success": True, "message": "Returning to launch"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/move', methods=['POST'])
def move():
    if not vehicle:
        return _no_vehicle()
    if not _cmd_lock.acquire(blocking=False):
        return jsonify({"success": False, "message": "Another command is in progress"}), 409

    try:
        data      = request.json or {}
        direction = data.get('direction')
        distance  = float(data.get('distance', 1))
        speed     = float(data.get('speed', 0.3))

        allowed = ['forward', 'backward', 'left', 'right', 'up', 'down']

        if direction not in allowed:
            _cmd_lock.release()
            return jsonify({"success": False, "message": f"Direction must be one of {allowed}"}), 400

        if speed < 0.2 or speed > 0.3:
            _cmd_lock.release()
            return jsonify({"success": False, "message": "Speed must be 0.2–0.3 m/s (indoor AI limit)"}), 400

        if not vehicle.armed:
            _cmd_lock.release()
            return jsonify({"success": False, "message": "Drone not armed — takeoff first"}), 400

        direction_to_mc = {
            'forward':  lambda: mc.move(north_m=distance,  speed=speed),
            'backward': lambda: mc.move(north_m=-distance, speed=speed),
            'left':     lambda: mc.move(east_m=-distance,  speed=speed),
            'right':    lambda: mc.move(east_m=distance,   speed=speed),
            'up':       lambda: mc.move(down_m=-distance,  speed=speed),
            'down':     lambda: mc.move(down_m=distance,   speed=speed),
        }

        move_fn = direction_to_mc[direction]

        def _move_worker():
            try:
                move_fn()
            finally:
                _cmd_lock.release()

        thread = threading.Thread(target=_move_worker, daemon=True)
        thread.start()

        return jsonify({
            "success":   True,
            "message":   f"Moving {direction} {distance}m at {speed}m/s",
            "direction": direction,
            "distance":  distance,
            "speed":     speed,
        })
    except Exception as e:
        _cmd_lock.release()
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/yaw', methods=['POST'])
def yaw():
    if not vehicle:
        return _no_vehicle()
    if not _cmd_lock.acquire(blocking=False):
        return jsonify({"success": False, "message": "Another command is in progress"}), 409

    try:
        data      = request.json or {}
        direction = data.get('direction', 'right')
        degrees   = float(data.get('degrees', 90))
        speed     = float(data.get('speed', 30))

        if direction not in ['left', 'right']:
            _cmd_lock.release()
            return jsonify({"success": False, "message": "Direction must be left or right"}), 400

        if degrees > 360:
            _cmd_lock.release()
            return jsonify({"success": False, "message": "Max rotation is 360 degrees"}), 400

        spin = 1 if direction == 'right' else -1

        # Lock released inside _send_yaw_locked (in finally block)
        thread = threading.Thread(
            target=_send_yaw_locked,
            args=(degrees, speed, spin, 1)
        )
        thread.daemon = True
        thread.start()

        return jsonify({
            "success":   True,
            "message":   f"Yawing {direction} {degrees} degrees",
            "direction": direction,
            "degrees":   degrees,
            "speed":     speed,
        })
    except Exception as e:
        _cmd_lock.release()
        return jsonify({"success": False, "message": str(e)}), 500

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
        mc.emergency_stop()
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
        mm.run_mission(steps, blocking=False)
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

@app.route('/video_feed', methods=['GET'])
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
        stop_camera()
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

@socketio.on('disconnect')
def on_disconnect():
    print("Client disconnected!")

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
    print("POST /move           {direction, distance, speed}")
    print("POST /yaw            {direction, degrees, speed}")
    print("POST /mode           {mode}")
    print("GET  /param?name=X")
    print("POST /param          {param, value}")
    print("POST /emergency")
    print("GET  /video_feed")
    print("POST /camera/start")
    print("POST /camera/stop")
    print("POST /queue/add      {action, params}")
    print("GET  /queue/status")
    print("POST /queue/clear")
    socketio.run(
        app,
        host='0.0.0.0',
        port=5000,
        debug=False,
        allow_unsafe_werkzeug=True
    )
