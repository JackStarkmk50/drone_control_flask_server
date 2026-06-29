# Save as takeoff_test.py
# USE IN OPEN AREA WITH PROPS ON

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

def arm_and_takeoff(vehicle, target_alt):
    
    #print("Checking if armable...")l
    #while not vehicle.is_armable:
    #    print("Waiting for vehicle to initialise...")
    #    time.sleep(1)

    print("Arming motors")
    #vehicle.mode = VehicleMode("GUIDED")
    vehicle.mode = VehicleMode("STABILIZE")
    print(f"Vehicle mode set to : ", vehicle.mode)
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off to", target_alt, "meters")
    vehicle.simple_takeoff(target_alt)

    # Wait until target altitude reached
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print("Altitude:", current_alt)
        
        # Break when 95% of target altitude reached
        if current_alt >= target_alt * 0.95:
            print("Target altitude reached!")
            break
        time.sleep(1)

# Connect
vehicle = connect('/dev/ttyAMA0',
                  baud=57600,
                  wait_ready=True)

# Takeoff to 2 meters
arm_and_takeoff(vehicle, 2)

# Hover for 5 seconds
print("Hovering for 5 seconds...")
time.sleep(5)

# Land
print("Landing...")
vehicle.mode = VehicleMode("LAND")

# Wait for landing
while vehicle.location.global_relative_frame.alt > 0.1:
    print("Altitude:", vehicle.location.global_relative_frame.alt)
    time.sleep(1)

print("Landed!")
vehicle.armed = False
vehicle.close()
