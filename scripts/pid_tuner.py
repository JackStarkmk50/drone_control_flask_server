# Save as scripts/pid_tuner.py

import eventlet
eventlet.monkey_patch()

from flask import (Flask, request,
                   jsonify, render_template)
from flask_socketio import SocketIO
from flask_cors import CORS
from dronekit import connect, VehicleMode
import threading
import time
import json
import os

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

# ─── PID Parameters Group ────────────────
PID_PARAMS = {
    "Roll Rate": {
        "ATC_RAT_RLL_P": "Roll Rate P",
        "ATC_RAT_RLL_I": "Roll Rate I",
        "ATC_RAT_RLL_D": "Roll Rate D",
        "ATC_RAT_RLL_IMAX": "Roll Rate IMAX",
        "ATC_RAT_RLL_FLTD": "Roll Rate Filter D",
    },
    "Pitch Rate": {
        "ATC_RAT_PIT_P": "Pitch Rate P",
        "ATC_RAT_PIT_I": "Pitch Rate I",
        "ATC_RAT_PIT_D": "Pitch Rate D",
        "ATC_RAT_PIT_IMAX": "Pitch Rate IMAX",
        "ATC_RAT_PIT_FLTD": "Pitch Rate Filter D",
    },
    "Yaw Rate": {
        "ATC_RAT_YAW_P": "Yaw Rate P",
        "ATC_RAT_YAW_I": "Yaw Rate I",
        "ATC_RAT_YAW_D": "Yaw Rate D",
        "ATC_RAT_YAW_IMAX": "Yaw Rate IMAX",
    },
    "Stabilize": {
        "ATC_ANG_RLL_P": "Stabilize Roll P",
        "ATC_ANG_PIT_P": "Stabilize Pitch P",
        "ATC_ANG_YAW_P": "Stabilize Yaw P",
    },
    "Altitude Hold": {
        "PSC_ACCZ_P": "Accel Z P",
        "PSC_ACCZ_I": "Accel Z I",
        "PSC_ACCZ_D": "Accel Z D",
        "PSC_VELZ_P": "Velocity Z P",
        "PSC_VELZ_I": "Velocity Z I",
        "PILOT_SPEED_UP": "Pilot Speed Up",
        "PILOT_SPEED_DN": "Pilot Speed Down",
    },
    "Loiter": {
        "PSC_POSXY_P": "Position XY P",
        "PSC_VELXY_P": "Velocity XY P",
        "PSC_VELXY_I": "Velocity XY I",
        "PSC_VELXY_D": "Velocity XY D",
        "LOIT_SPEED":  "Loiter Speed",
        "LOIT_ACC_MAX": "Loiter Accel Max",
        "LOIT_ANG_MAX": "Loiter Angle Max",
    },
    "Motor": {
        "MOT_THST_HOVER": "Hover Thrust",
        "MOT_THST_EXPO": "Thrust Expo",
        "MOT_SPIN_ARM": "Spin When Armed",
        "MOT_SPIN_MIN": "Spin Minimum",
        "MOT_SPIN_MAX": "Spin Maximum",
    },
    "Filters": {
        "INS_GYRO_FILTER": "Gyro Filter Hz",
        "INS_ACCEL_FILTER": "Accel Filter Hz",
        "ATC_RAT_RLL_FLTT": "Roll Filter T",
        "ATC_RAT_PIT_FLTT": "Pitch Filter T",
    },
}

# ─── Connect Drone ───────────────────────
def connect_drone():
    global vehicle
    print("Connecting to Pixhawk...")
    try:
        vehicle = connect(
            '/dev/ttyAMA0',
            baud=57600,
            wait_ready=True
        )
        print("Connected!")
    except Exception as e:
        print(f"Connection failed: {e}")

# ─── Telemetry Stream ────────────────────
def stream_telemetry():
    while True:
        try:
            if vehicle:
                data = {
                    "mode": vehicle.mode.name,
                    "armed": vehicle.armed,
                    "altitude": round(
                        vehicle.location
                        .global_relative_frame
                        .alt, 2),
                    "battery": round(
                        vehicle.battery.voltage,
                        1),
                    "pitch": round(
                        vehicle.attitude.pitch,
                        3),
                    "roll": round(
                        vehicle.attitude.roll,
                        3),
                    "yaw": round(
                        vehicle.attitude.yaw,
                        3),
                    "satellites": vehicle.gps_0
                        .satellites_visible,
                    "gps_fix": vehicle.gps_0
                        .fix_type,
                    "ekf_ok": vehicle.ekf_ok,
                    "groundspeed": round(
                        vehicle.groundspeed, 1),
                }
                socketio.emit('telemetry', data)
        except Exception as e:
            print(f"Telemetry error: {e}")
        time.sleep(0.2)

# ─── Routes ──────────────────────────────

@app.route('/')
def index():
    return render_template('pid_tuner.html')

# Get all PID params at once
@app.route('/pid/all', methods=['GET'])
def get_all_pids():
    if not vehicle:
        return jsonify({
            "error": "Not connected"
        }), 500

    result = {}
    for group, params in PID_PARAMS.items():
        result[group] = {}
        for param, label in params.items():
            try:
                value = vehicle.parameters[param]
                result[group][param] = {
                    "label": label,
                    "value": round(float(value), 6)
                }
            except Exception as e:
                result[group][param] = {
                    "label": label,
                    "value": None,
                    "error": str(e)
                }
    return jsonify(result)

# Get single param
@app.route('/pid/get', methods=['GET'])
def get_pid():
    param = request.args.get('param')
    if not param:
        return jsonify({
            "error": "param required"
        }), 400
    try:
        value = vehicle.parameters[param]
        return jsonify({
            "param": param,
            "value": round(float(value), 6)
        })
    except Exception as e:
        return jsonify({
            "error": str(e)
        }), 404

# Set single param
@app.route('/pid/set', methods=['POST'])
def set_pid():
    data  = request.json
    param = data.get('param')
    value = float(data.get('value'))

    if not param or value is None:
        return jsonify({
            "error": "param and value required"
        }), 400

    try:
        old_value = float(
            vehicle.parameters[param])
        vehicle.parameters[param] = value
        time.sleep(0.2)

        # Verify it was set
        new_value = float(
            vehicle.parameters[param])

        return jsonify({
            "success":   True,
            "param":     param,
            "old_value": round(old_value, 6),
            "new_value": round(new_value, 6),
            "message":   f"{param} changed "
                        f"{old_value:.4f} → "
                        f"{new_value:.4f}"
        })
    except Exception as e:
        return jsonify({
            "error": str(e)
        }), 500

# Set multiple params at once
@app.route('/pid/setmany', methods=['POST'])
def set_many_pids():
    data   = request.json
    params = data.get('params', {})
    results = []

    for param, value in params.items():
        try:
            old = float(vehicle.parameters[param])
            vehicle.parameters[param] = float(value)
            time.sleep(0.1)
            new = float(vehicle.parameters[param])
            results.append({
                "param":     param,
                "success":   True,
                "old_value": round(old, 6),
                "new_value": round(new, 6),
            })
        except Exception as e:
            results.append({
                "param":   param,
                "success": False,
                "error":   str(e)
            })

    return jsonify({
        "success": True,
        "results": results
    })

# Reset single param to default
@app.route('/pid/reset', methods=['POST'])
def reset_pid():
    data  = request.json
    param = data.get('param')

    defaults = {
        "ATC_RAT_RLL_P":   0.135,
        "ATC_RAT_RLL_I":   0.135,
        "ATC_RAT_RLL_D":   0.0036,
        "ATC_RAT_PIT_P":   0.135,
        "ATC_RAT_PIT_I":   0.135,
        "ATC_RAT_PIT_D":   0.0036,
        "ATC_RAT_YAW_P":   0.18,
        "ATC_RAT_YAW_I":   0.018,
        "ATC_ANG_RLL_P":   4.5,
        "ATC_ANG_PIT_P":   4.5,
        "ATC_ANG_YAW_P":   4.5,
        "PSC_ACCZ_P":      0.5,
        "PSC_ACCZ_I":      1.0,
        "PSC_ACCZ_D":      0.0,
        "INS_GYRO_FILTER": 20,
        "MOT_THST_HOVER":  0.35,
    }

    if param not in defaults:
        return jsonify({
            "error": "No default known for this param"
        }), 400

    try:
        default_val = defaults[param]
        vehicle.parameters[param] = default_val
        return jsonify({
            "success":       True,
            "param":         param,
            "default_value": default_val
        })
    except Exception as e:
        return jsonify({
            "error": str(e)
        }), 500

# Save current params to file
@app.route('/pid/save', methods=['POST'])
def save_params():
    data     = request.json or {}
    filename = data.get(
        'filename', 'pid_backup.json')
    filepath = os.path.join(
        '/home/virtua/drone/config',
        filename)

    try:
        all_params = {}
        for group, params in PID_PARAMS.items():
            for param in params:
                try:
                    val = vehicle.parameters[param]
                    all_params[param] = float(val)
                except:
                    pass

        with open(filepath, 'w') as f:
            json.dump(all_params, f, indent=2)

        return jsonify({
            "success":  True,
            "message":  f"Saved to {filename}",
            "filepath": filepath,
            "count":    len(all_params)
        })
    except Exception as e:
        return jsonify({
            "error": str(e)
        }), 500

# Load params from saved file
@app.route('/pid/load', methods=['POST'])
def load_params():
    data     = request.json or {}
    filename = data.get(
        'filename', 'pid_backup.json')
    filepath = os.path.join(
        '/home/virtua/drone/config',
        filename)

    try:
        with open(filepath, 'r') as f:
            params = json.load(f)

        results = []
        for param, value in params.items():
            try:
                vehicle.parameters[param] = value
                time.sleep(0.1)
                results.append({
                    "param":   param,
                    "value":   value,
                    "success": True
                })
            except Exception as e:
                results.append({
                    "param":   param,
                    "error":   str(e),
                    "success": False
                })

        return jsonify({
            "success": True,
            "loaded":  len(results),
            "results": results
        })
    except FileNotFoundError:
        return jsonify({
            "error": f"File {filename} not found"
        }), 404

# List saved param files
@app.route('/pid/files', methods=['GET'])
def list_files():
    config_dir = '/home/virtua/drone/config'
    try:
        files = [f for f in os.listdir(config_dir)
                 if f.endswith('.json')]
        return jsonify({
            "files": files
        })
    except Exception as e:
        return jsonify({
            "error": str(e)
        }), 500

# Network settings page
@app.route('/network')
def network_page():
    return render_template('network.html')

# Scan available WiFi networks
@app.route('/network/scan', methods=['GET'])
def scan_networks():
    import subprocess
    try:
        result = subprocess.run(
            ['sudo', 'iwlist', 'wlan0', 'scan'],
            capture_output=True,
            text=True
        )
        networks = []
        current = {}
        for line in result.stdout.split('\n'):
            line = line.strip()
            if 'ESSID:' in line:
                ssid = line.split('"')[1]
                if ssid:
                    current['ssid'] = ssid
                    networks.append(
                        current.copy())
                    current = {}
            elif 'Signal level=' in line:
                try:
                    signal = line.split(
                        'Signal level=')[1]\
                        .split(' ')[0]
                    current['signal'] = signal
                except:
                    pass

        return jsonify({
            "networks": networks
        })
    except Exception as e:
        return jsonify({
            "error": str(e)
        }), 500

# Connect to WiFi network
@app.route('/network/connect', methods=['POST'])
def connect_network():
    import subprocess
    data     = request.json
    ssid     = data.get('ssid')
    password = data.get('password')

    if not ssid:
        return jsonify({
            "error": "SSID required"
        }), 400

    try:
        # Add network to wpa_supplicant
        config = f'''
network={{
    ssid="{ssid}"
    psk="{password}"
    key_mgmt=WPA-PSK
}}'''
        with open('/tmp/wifi_add.conf', 'w') as f:
            f.write(config)

        subprocess.run([
            'sudo', 'bash', '-c',
            f'cat /tmp/wifi_add.conf >> '
            f'/etc/wpa_supplicant/'
            f'wpa_supplicant.conf'
        ])

        subprocess.run([
            'sudo', 'wpa_cli',
            '-i', 'wlan0', 'reconfigure'
        ])

        return jsonify({
            "success": True,
            "message": f"Connecting to {ssid}..."
        })
    except Exception as e:
        return jsonify({
            "error": str(e)
        }), 500

# ─── WebSocket ───────────────────────────
@socketio.on('connect')
def on_connect():
    print("Client connected!")
    t = threading.Thread(
        target=stream_telemetry)
    t.daemon = True
    t.start()

@socketio.on('disconnect')
def on_disconnect():
    print("Client disconnected!")

# ─── Main ────────────────────────────────
if __name__ == '__main__':
    # Create config dir if not exists
    os.makedirs(
        '/home/virtua/drone/config',
        exist_ok=True)

    connect_drone()
    print("\nPID Tuner running!")
    print("Connect to WiFi: DroneControl")
    print("Password: drone1234")
    print("Open browser: http://192.168.4.1:5000")
    socketio.run(
        app,
        host='0.0.0.0',
        port=5000,
        debug=False
    )
