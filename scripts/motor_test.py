# REMOVE ALL PROPS FIRST!

from dronekit import connect, VehicleMode
import time

vehicle = connect('/dev/ttyAMA0', 
                  baud=57600,
                  wait_ready=True)

print("Connected to Pixhawk")

# Set to GUIDED mode
#vehicle.mode = VehicleMode("GUIDED")
vehicle.mode = VehicleMode("STABILIZE")
print(f"Vehicle mode set to",vehicle.mode)
time.sleep(1)

# Arm the vehicle
print("Arming...")
vehicle.armed = True

# Wait for arming
while not vehicle.armed:
    print("Waiting for arm...")
    time.sleep(1)

print("ARMED! Motors should spin")
time.sleep(3)

# Disarm
vehicle.armed = False
print("Disarmed")
vehicle.close()
