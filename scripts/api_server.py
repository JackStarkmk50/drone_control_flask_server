# Save as scripts/api_server.py

from flask import Flask, request, jsonify
from flask_socketio import SocketIO, emit
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import anthropic
import threading
import time
import json

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# ─── Global vehicle object ───────────────
vehicle = None
telemetry_active = False

# ─── Connect to Drone ────────────────────
def connect_drone():
    global vehicle
    print("Connecting to Pixhawk...")
    vehicle = connect('/dev/ttyAMA0',
                     baud=57600,
                     wait_ready=True)
    print("Drone connected!")

# ─── Telemetry Stream ────────────────────
def stream_telemetry():
    global telemetry_active
    telemetry_active = True
    
    while telemetry_active:
        if vehicle:
            data = {
                "mode": vehicle.mode.name,
                "armed": vehicle.armed,
                "altitude": round(
                    vehicle.location.global_relative_frame.alt, 2),
                "battery": round(vehicle.battery.voltage, 1),
                "battery_level": vehicle.battery.level,
                "satellites": vehicle.gps_0.satellites_visible,
                "gps_fix": vehicle.gps_0.fix_type,
                "pitch": round(vehicle.attitude.pitch, 3),
                "roll": round(vehicle.attitude.roll, 3),
                "yaw": round(vehicle.attitude.yaw, 3),
                "lat": vehicle.location.global_frame.lat,
                "lon": vehicle.location.global_frame.lon,
                "airspeed": round(vehicle.airspeed, 1),
                "groundspeed": round(vehicle.groundspeed, 1),
                "is_armable": vehicle.is_armable,
                "ekf_ok": vehicle.ekf_ok,
            }
            socketio.emit('telemetry', data)
        time.sleep(0.2)

# ─── Drone Command Functions ─────────────
def arm_and_takeoff(altitude):
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)
    vehicle.armed = True
    start = time.time()
    while not vehicle.armed:
        if time.time() - start > 15:
            print("Arm timeout!")
            return False
        time.sleep(0.5)
    vehicle.simple_takeoff(altitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt >= altitude * 0.95:
            break
        time.sleep(0.5)
    return True

def send_velocity(vx, vy, vz, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
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
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

# ─── REST API Endpoints ──────────────────

@app.route('/status', methods=['GET'])
def get_status():
    if not vehicle:
        return jsonify({"error": "Drone not connected"}), 500
    return jsonify({
        "mode": vehicle.mode.name,
        "armed": vehicle.armed,
        "altitude": round(
            vehicle.location.global_relative_frame.alt, 2),
        "battery": round(vehicle.battery.voltage, 1),
        "satellites": vehicle.gps_0.satellites_visible,
        "gps_fix": vehicle.gps_0.fix_type,
        "pitch": round(vehicle.attitude.pitch, 3),
        "roll": round(vehicle.attitude.roll, 3),
        "yaw": round(vehicle.attitude.yaw, 3),
        "lat": vehicle.location.global_frame.lat,
        "lon": vehicle.location.global_frame.lon,
        "is_armable": vehicle.is_armable,
        "ekf_ok": vehicle.ekf_ok,
    })

@app.route('/takeoff', methods=['POST'])
def takeoff():
    data = request.json
    altitude = data.get('altitude', 2)
    
    if vehicle.battery.voltage < 10.5:
        return jsonify({
            "success": False,
            "error": "Battery too low"
        }), 400
    
    #if vehicle.gps_0.fix_type < 3:
    #    return jsonify({
    #        "success": False,
    #        "error": "No GPS fix"
    #    }), 400
    
    if altitude > 10:
        return jsonify({
            "success": False,
            "error": "Altitude too high (max 10m)"
        }), 400
    
    thread = threading.Thread(
        target=arm_and_takeoff,
        args=(altitude,))
    thread.start()
    
    return jsonify({
        "success": True,
        "message": f"Taking off to {altitude}m"
    })

@app.route('/land', methods=['POST'])
def land():
    vehicle.mode = VehicleMode("LAND")
    return jsonify({
        "success": True,
        "message": "Landing"
    })

@app.route('/rtl', methods=['POST'])
def rtl():
    vehicle.mode = VehicleMode("RTL")
    return jsonify({
        "success": True,
        "message": "Returning to launch"
    })

@app.route('/move', methods=['POST'])
def move():
    data = request.json
    direction = data.get('direction')
    distance = data.get('distance', 1)
    speed = data.get('speed', 0.5)
    duration = distance / speed

    directions = {
        'forward':  (speed,  0,     0),
        'backward': (-speed, 0,     0),
        'left':     (0,     -speed, 0),
        'right':    (0,      speed, 0),
        'up':       (0,      0,    -speed),
        'down':     (0,      0,     speed),
    }

    if direction not in directions:
        return jsonify({
            "success": False,
            "error": "Invalid direction"
        }), 400

    vx, vy, vz = directions[direction]
    thread = threading.Thread(
        target=send_velocity,
        args=(vx, vy, vz, duration))
    thread.start()

    return jsonify({
        "success": True,
        "message": f"Moving {direction} {distance}m"
    })

@app.route('/mode', methods=['POST'])
def set_mode():
    data = request.json
    mode = data.get('mode', 'STABILIZE')
    
    allowed_modes = [
        'STABILIZE', 'ALTHOLD',
        'LOITER', 'GUIDED',
        'LAND', 'RTL'
    ]
    
    if mode not in allowed_modes:
        return jsonify({
            "success": False,
            "error": "Invalid mode"
        }), 400
    
    vehicle.mode = VehicleMode(mode)
    return jsonify({
        "success": True,
        "message": f"Mode set to {mode}"
    })

@app.route('/param', methods=['GET'])
def get_param():
    param = request.args.get('name')
    value = vehicle.parameters[param]
    return jsonify({
        "param": param,
        "value": value
    })

@app.route('/param', methods=['POST'])
def set_param():
    data = request.json
    param = data.get('param')
    value = data.get('value')
    vehicle.parameters[param] = value
    return jsonify({
        "success": True,
        "param": param,
        "value": value
    })

# ─── LLM Command Endpoint ────────────────

client = anthropic.Anthropic(
    api_key="YOUR_ANTHROPIC_API_KEY")

SYSTEM_PROMPT = """
You are a drone flight controller AI.
You control a real drone.
You receive drone status and user command.

Respond ONLY with JSON:
{
  "action": "action_name",
  "params": {},
  "message": "explanation",
  "safe": true/false
}

Actions available:
takeoff, land, rtl, move, hold,
set_mode, status, none

Safety rules:
- Never takeoff if battery under 10.5V
- Never exceed 10m altitude
- Never fly if GPS fix under 3
- Always land if battery under 11V
- Mark safe=false if unsafe
"""

@app.route('/command', methods=['POST'])
def llm_command():
    data = request.json
    user_command = data.get('command', '')
    
    # Get current status
    status = {
        "mode": vehicle.mode.name,
        "armed": vehicle.armed,
        "altitude": round(
            vehicle.location.global_relative_frame.alt, 2),
        "battery": round(vehicle.battery.voltage, 1),
        "satellites": vehicle.gps_0.satellites_visible,
        "gps_fix": vehicle.gps_0.fix_type,
        "is_armable": vehicle.is_armable,
    }
    
    # Ask LLM
    response = client.messages.create(
        model="claude-sonnet-4-6",
        max_tokens=500,
        system=SYSTEM_PROMPT,
        messages=[{
            "role": "user",
            "content": f"Status: {status}\nCommand: {user_command}"
        }]
    )
    
    llm_response = json.loads(
        response.content[0].text)
    
    # Execute if safe
    if llm_response.get('safe', False):
        action = llm_response.get('action')
        params = llm_response.get('params', {})
        
        if action == 'takeoff':
            threading.Thread(
                target=arm_and_takeoff,
                args=(params.get('altitude', 2),)
            ).start()
        
        elif action == 'land':
            vehicle.mode = VehicleMode("LAND")
        
        elif action == 'rtl':
            vehicle.mode = VehicleMode("RTL")
        
        elif action == 'hold':
            vehicle.mode = VehicleMode("LOITER")
        
        elif action == 'move':
            direction = params.get('direction')
            distance = params.get('distance', 1)
            speed = params.get('speed', 0.5)
            duration = distance / speed
            
            directions = {
                'forward':  (speed,  0,     0),
                'backward': (-speed, 0,     0),
                'left':     (0,     -speed, 0),
                'right':    (0,      speed, 0),
                'up':       (0,      0,    -speed),
                'down':     (0,      0,     speed),
            }
            
            if direction in directions:
                vx, vy, vz = directions[direction]
                threading.Thread(
                    target=send_velocity,
                    args=(vx, vy, vz, duration)
                ).start()
    
    return jsonify({
        "success": True,
        "llm_response": llm_response,
        "command": user_command,
        "status": status
    })

# ─── WebSocket Events ────────────────────

@socketio.on('connect')
def on_connect():
    print("Client connected")
    thread = threading.Thread(
        target=stream_telemetry)
    thread.daemon = True
    thread.start()

@socketio.on('disconnect')
def on_disconnect():
    global telemetry_active
    telemetry_active = False
    print("Client disconnected")

# ─── Start Server ─────────────────────────

if __name__ == '__main__':
    connect_drone()
    print("Starting API server...")
    print("REST API: http://localhost:5000")
    print("WebSocket: ws://localhost:5000")
    socketio.run(app,
                host='0.0.0.0',
                port=5000,
                debug=False)
