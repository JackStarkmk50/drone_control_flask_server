import eventlet
eventlet.monkey_patch()

from flask import Flask, render_template
from flask_socketio import SocketIO
from dronekit import connect
import threading
import time

app = Flask(__name__,
            template_folder='../templates',
            static_folder='../static')

socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode='eventlet'
)

# Connect drone
print("Connecting to Pixhawk...")
vehicle = connect('/dev/ttyAMA0',
                 baud=57600,
                 wait_ready=True)
print("Connected!")

# Serve HTML page
@app.route('/')
def index():
    return render_template('index.html')

# Telemetry stream
def stream_telemetry():
    while True:
        if vehicle:
            data = {
                "mode": vehicle.mode.name,
                "armed": vehicle.armed,
                "altitude": round(
                    vehicle.location
                    .global_relative_frame.alt, 2),
                "battery": round(
                    vehicle.battery.voltage, 1),
                "battery_level":
                    vehicle.battery.level,
                "satellites":
                    vehicle.gps_0.satellites_visible,
                "gps_fix":
                    vehicle.gps_0.fix_type,
                "pitch": round(
                    vehicle.attitude.pitch, 3),
                "roll": round(
                    vehicle.attitude.roll, 3),
                "yaw": round(
                    vehicle.attitude.yaw, 3),
                "lat": vehicle.location
                    .global_frame.lat,
                "lon": vehicle.location
                    .global_frame.lon,
                "groundspeed": round(
                    vehicle.groundspeed, 1),
                "airspeed": round(
                    vehicle.airspeed, 1),
                "ekf_ok": vehicle.ekf_ok,
            }
            socketio.emit('telemetry', data)
        time.sleep(0.1)

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

if __name__ == '__main__':
    print("Server starting...")
    print("Open: http://localhost:5000")
    socketio.run(
        app,
        host='0.0.0.0',
        port=5000,
        debug=False
    )
