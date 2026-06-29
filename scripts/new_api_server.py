import collections
import eventlet
eventlet.monkey_patch()

from flask import Flask, request, jsonify, render_template
from flask_socketio import SocketIO
from flask_cors import CORS
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import threading
import time

app = Flask(__name__,
            template_folder='../templates',
            static_folder='../static')

CORS(app)
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode='eventlet'
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

_vibe = {"x": 0.0, "y": 0.0, "z": 0.0}

# ─── Connect Drone ───────────────────────
def connect_drone():
    global vehicle
    print("Connecting to Pixhawk...")
    vehicle = connect(
        '/dev/ttyAMA0',
        baud=57600,
        wait_ready=True
    )

    @vehicle.on_message('VIBRATION')
    def on_vibration(self, name, message):
        _vibe["x"] = round(message.vibration_x, 3)
        _vibe["y"] = round(message.vibration_y, 3)
        _vibe["z"] = round(message.vibration_z, 3)

    print("Drone connected!")

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
        ekf_ok    = bool(vehicle.ekf_ok)
        is_armable = bool(vehicle.is_armable)
    except Exception:
        ekf_ok = is_armable = False

    return {
        "mode":          mode,
        "armed":         armed,
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
        "vibe_x":        _vibe["x"],
        "vibe_y":        _vibe["y"],
        "vibe_z":        _vibe["z"],
    }

def send_velocity(vx, vy, vz, duration):
    msg = vehicle.message_factory\
        .set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    start = time.time()
    while time.time() - start < duration:
        vehicle.send_mavlink(msg)
        time.sleep(0.1)
    send_stop()

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

def arm_and_takeoff(altitude):
    """Runs in daemon thread. Releases _cmd_lock when done."""
    try:
        if vehicle.gps_0.fix_type < 3:
            vehicle.mode = VehicleMode("GUIDED_NOGPS")
        else:
            vehicle.mode = VehicleMode("GUIDED")

        time.sleep(1)

        vehicle.armed = True
        start = time.time()
        while not vehicle.armed:
            if time.time() - start > 15:
                print("Arm failed!")
                return
            print("Waiting for arm...")
            time.sleep(0.5)

        print("Armed!")

        if vehicle.gps_0.fix_type >= 3:
            vehicle.simple_takeoff(altitude)
            while True:
                alt = vehicle.location\
                    .global_relative_frame.alt
                print(f"Altitude: {alt:.1f}m")
                if alt >= altitude * 0.95:
                    break
                time.sleep(0.5)
        else:
            print("No GPS - using thrust control")
            hover_thrust = 0.68
            climb_thrust = hover_thrust + 0.08
            target_alt   = altitude

            start = time.time()
            while True:
                current_alt = safe_float(
                    vehicle.location.global_relative_frame.alt)

                msg = vehicle.message_factory\
                    .set_attitude_target_encode(
                    0,
                    1, 1,
                    0b00000111,
                    [1, 0, 0, 0],
                    0, 0, 0,
                    climb_thrust
                )
                vehicle.send_mavlink(msg)
                print(f"Alt: {current_alt:.1f}/{target_alt}m")

                if current_alt >= target_alt * 0.90:
                    print("Target altitude reached!")
                    break

                if time.time() - start > 30:
                    print("Takeoff timeout!")
                    vehicle.mode = VehicleMode("LAND")
                    return

                time.sleep(0.1)

            for i in range(20):
                msg = vehicle.message_factory\
                    .set_attitude_target_encode(
                    0,
                    1, 1,
                    0b00000111,
                    [1, 0, 0, 0],
                    0, 0, 0,
                    hover_thrust
                )
                vehicle.send_mavlink(msg)
                time.sleep(0.1)
    finally:
        _cmd_lock.release()

def _send_velocity_locked(vx, vy, vz, duration):
    """Wrapper that releases _cmd_lock after send_velocity completes."""
    try:
        send_velocity(vx, vy, vz, duration)
    finally:
        _cmd_lock.release()

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
        time.sleep(0.2)

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

    return jsonify({
        "connected":     True,
        "armed":         bool(vehicle.armed),
        "mode":          vehicle.mode.name if vehicle.mode else "UNKNOWN",
        "battery":       safe_round(vehicle.battery.voltage, 1),
        "gps_fix":       safe_int(vehicle.gps_0.fix_type),
        "system_status": sys_status,
    })

@app.route('/lvlcal', methods=['POST'])
def lvlcal():
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
    try:
        if vehicle.armed:
            return jsonify({"success": False, "message": "Already armed"})

        vehicle.armed = True
        start = time.time()
        while not vehicle.armed:
            if time.time() - start > 15:
                return jsonify({"success": False, "message": "Arm timeout"}), 500
            time.sleep(0.5)

        return jsonify({"success": True, "message": "Armed successfully"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/disarm', methods=['POST'])
def disarm():
    try:
        if not vehicle.armed:
            return jsonify({"success": False, "message": "Already disarmed"})

        vehicle.armed = False
        time.sleep(1)
        return jsonify({"success": True, "message": "Disarmed successfully"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/safety', methods=['POST'])
def safety():
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
    # FIX 5: acquire command lock before starting flight action
    if not _cmd_lock.acquire(blocking=False):
        return jsonify({"success": False, "message": "Another command is in progress"}), 409

    try:
        data     = request.json or {}
        altitude = float(data.get('altitude', 2))

        if safe_float(vehicle.battery.voltage) < 10.5:
            _cmd_lock.release()
            return jsonify({"success": False, "message": "Battery too low"}), 400

        if altitude > 10:
            _cmd_lock.release()
            return jsonify({"success": False, "message": "Max altitude is 10m"}), 400

        if altitude < 0.5:
            _cmd_lock.release()
            return jsonify({"success": False, "message": "Min altitude is 0.5m"}), 400

        # Lock released inside arm_and_takeoff (in finally block)
        thread = threading.Thread(target=arm_and_takeoff, args=(altitude,))
        thread.daemon = True
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
    if not _cmd_lock.acquire(blocking=False):
        return jsonify({"success": False, "message": "Another command is in progress"}), 409
    try:
        vehicle.mode = VehicleMode("LAND")
        return jsonify({"success": True, "message": "Landing"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500
    finally:
        _cmd_lock.release()

@app.route('/rtl', methods=['POST'])
def rtl():
    if not _cmd_lock.acquire(blocking=False):
        return jsonify({"success": False, "message": "Another command is in progress"}), 409
    try:
        if safe_int(vehicle.gps_0.fix_type) < 3:
            return jsonify({"success": False, "message": "No GPS fix for RTL"}), 400
        vehicle.mode = VehicleMode("RTL")
        return jsonify({"success": True, "message": "Returning to launch"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500
    finally:
        _cmd_lock.release()

@app.route('/move', methods=['POST'])
def move():
    if not _cmd_lock.acquire(blocking=False):
        return jsonify({"success": False, "message": "Another command is in progress"}), 409

    try:
        data      = request.json or {}
        direction = data.get('direction')
        distance  = float(data.get('distance', 1))
        speed     = float(data.get('speed', 0.5))

        allowed = ['forward', 'backward', 'left', 'right', 'up', 'down']

        if direction not in allowed:
            _cmd_lock.release()
            return jsonify({"success": False, "message": f"Direction must be one of {allowed}"}), 400

        if speed > 3.0:
            _cmd_lock.release()
            return jsonify({"success": False, "message": "Max speed is 3.0 m/s"}), 400

        duration = distance / speed

        direction_map = {
            'forward':  ( speed,   0,      0),
            'backward': (-speed,   0,      0),
            'left':     ( 0,      -speed,  0),
            'right':    ( 0,       speed,  0),
            'up':       ( 0,       0,     -speed),
            'down':     ( 0,       0,      speed),
        }

        vx, vy, vz = direction_map[direction]

        # Lock released inside _send_velocity_locked (in finally block)
        thread = threading.Thread(
            target=_send_velocity_locked,
            args=(vx, vy, vz, duration)
        )
        thread.daemon = True
        thread.start()

        return jsonify({
            "success":   True,
            "message":   f"Moving {direction} {distance}m at {speed}m/s",
            "direction": direction,
            "distance":  distance,
            "speed":     speed,
            "duration":  round(duration, 2),
        })
    except Exception as e:
        _cmd_lock.release()
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/yaw', methods=['POST'])
def yaw():
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
    try:
        if safe_int(vehicle.gps_0.fix_type) >= 3:
            vehicle.mode = VehicleMode("LOITER")
            return jsonify({"success": True, "message": "Holding position (Loiter)"})
        else:
            vehicle.mode = VehicleMode("ALTHOLD")
            return jsonify({"success": True, "message": "Holding altitude (no GPS)"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/emergency', methods=['POST'])
def emergency():
    # Emergency bypasses command lock intentionally
    try:
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        vehicle.armed = False
        return jsonify({"success": True, "message": "Emergency stop executed"})
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
    connect_drone()
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
    print("POST /queue/add      {action, params}")
    print("GET  /queue/status")
    print("POST /queue/clear")
    socketio.run(
        app,
        host='0.0.0.0',
        port=5000,
        debug=False
    )
