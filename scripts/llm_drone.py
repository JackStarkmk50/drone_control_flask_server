# Complete LLM controlled drone

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import anthropic
import time
import math

# ─── Connect to Drone ───────────────────────
print("Connecting to Pixhawk...")
vehicle = connect('/dev/ttyAMA0',
                  baud=57600,
                  wait_ready=True)
print("Connected!\n")

# ─── Drone Functions ────────────────────────

def arm_and_takeoff(altitude):
    print(f"Taking off to {altitude}m")
    
    while not vehicle.is_armable:
        print("Waiting for armable...")
        time.sleep(1)
    
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    while not vehicle.armed:
        time.sleep(0.5)
    
    vehicle.simple_takeoff(altitude)
    
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {alt:.1f}m")
        if alt >= altitude * 0.95:
            print("Target altitude reached!")
            break
        time.sleep(0.5)

def land():
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.location.global_relative_frame.alt > 0.2:
        time.sleep(0.5)
    vehicle.armed = False
    print("Landed!")

def hold_position():
    print("Holding position...")
    vehicle.mode = VehicleMode("LOITER")

def return_home():
    print("Returning to launch...")
    vehicle.mode = VehicleMode("RTL")

def send_velocity(vx, vy, vz, duration):
    # vx = forward/back (m/s)
    # vy = left/right (m/s)
    # vz = up/down (m/s) negative = up
    
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

def move_forward(distance_m, speed=0.5):
    print(f"Moving forward {distance_m}m")
    duration = distance_m / speed
    send_velocity(speed, 0, 0, duration)
    send_velocity(0, 0, 0, 0.5)

def move_backward(distance_m, speed=0.5):
    print(f"Moving backward {distance_m}m")
    duration = distance_m / speed
    send_velocity(-speed, 0, 0, duration)
    send_velocity(0, 0, 0, 0.5)

def move_left(distance_m, speed=0.5):
    print(f"Moving left {distance_m}m")
    duration = distance_m / speed
    send_velocity(0, -speed, 0, duration)
    send_velocity(0, 0, 0, 0.5)

def move_right(distance_m, speed=0.5):
    print(f"Moving right {distance_m}m")
    duration = distance_m / speed
    send_velocity(0, speed, 0, duration)
    send_velocity(0, 0, 0, 0.5)

def get_drone_status():
    return {
        "mode": vehicle.mode.name,
        "armed": vehicle.armed,
        "altitude": round(vehicle.location.global_relative_frame.alt, 2),
        "battery_voltage": round(vehicle.battery.voltage, 1),
        "battery_level": vehicle.battery.level,
        "satellites": vehicle.gps_0.satellites_visible,
        "gps_fix": vehicle.gps_0.fix_type,
        "pitch": round(vehicle.attitude.pitch, 3),
        "roll": round(vehicle.attitude.roll, 3),
        "yaw": round(vehicle.attitude.yaw, 3),
        "airspeed": round(vehicle.airspeed, 1),
        "is_armable": vehicle.is_armable,
    }

# ─── LLM Controller ─────────────────────────

client = anthropic.Anthropic(api_key="YOUR_API_KEY")

SYSTEM_PROMPT = """
You are a drone flight controller AI.
You control a real drone via DroneKit.
You receive the current drone status and
a command from the user.

You must respond with ONLY a JSON object:
{
  "action": "action_name",
  "params": {},
  "message": "what you are doing"
}

Available actions:
- takeoff: params: {"altitude": 2}
- land: params: {}
- hold: params: {}
- return_home: params: {}
- move_forward: params: {"distance": 1, "speed": 0.5}
- move_backward: params: {"distance": 1, "speed": 0.5}
- move_left: params: {"distance": 1, "speed": 0.5}
- move_right: params: {"distance": 1, "speed": 0.5}
- status: params: {}
- none: params: {} (if command unclear or unsafe)

Safety rules:
- Never arm if battery under 10.5V
- Never takeoff if GPS fix under 3
- Never exceed altitude 10m
- Always land if battery under 11V
- Refuse unsafe commands
"""

def ask_llm(user_command, drone_status):
    response = client.messages.create(
        model="claude-sonnet-4-20250514",
        max_tokens=500,
        system=SYSTEM_PROMPT,
        messages=[{
            "role": "user",
            "content": f"Drone status: {drone_status}\nCommand: {user_command}"
        }]
    )
    return response.content[0].text

def execute_action(action_data):
    import json
    
    try:
        data = json.loads(action_data)
        action = data.get("action")
        params = data.get("params", {})
        message = data.get("message", "")
        
        print(f"\nAI: {message}")
        
        if action == "takeoff":
            arm_and_takeoff(params.get("altitude", 2))
        
        elif action == "land":
            land()
        
        elif action == "hold":
            hold_position()
        
        elif action == "return_home":
            return_home()
        
        elif action == "move_forward":
            move_forward(
                params.get("distance", 1),
                params.get("speed", 0.5)
            )
        
        elif action == "move_backward":
            move_backward(
                params.get("distance", 1),
                params.get("speed", 0.5)
            )
        
        elif action == "move_left":
            move_left(
                params.get("distance", 1),
                params.get("speed", 0.5)
            )
        
        elif action == "move_right":
            move_right(
                params.get("distance", 1),
                params.get("speed", 0.5)
            )
        
        elif action == "status":
            status = get_drone_status()
            for key, val in status.items():
                print(f"  {key}: {val}")
        
        elif action == "none":
            print("AI: Command not executed (unclear or unsafe)")
            
    except Exception as e:
        print(f"Error executing action: {e}")

# ─── Main Loop ──────────────────────────────

print("LLM Drone Controller Ready!")
print("Type commands in natural language")
print("Examples:")
print("  'take off to 3 meters'")
print("  'move forward 2 meters'")
print("  'hold position'")
print("  'land now'")
print("Type 'quit' to exit\n")

try:
    while True:
        user_input = input("Command: ").strip()
        
        if user_input.lower() == 'quit':
            break
        
        if not user_input:
            continue
        
        # Get current drone status
        status = get_drone_status()
        
        # Ask LLM what to do
        print("Thinking...")
        llm_response = ask_llm(user_input, status)
        
        # Execute the action
        execute_action(llm_response)

except KeyboardInterrupt:
    print("\nShutting down...")

finally:
    if vehicle.armed:
        vehicle.mode = VehicleMode("LAND")
        time.sleep(3)
    vehicle.close()
    print("Disconnected safely")
