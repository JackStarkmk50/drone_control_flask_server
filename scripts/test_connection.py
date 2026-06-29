from dronekit import connect, VehicleMode
import time

print("Connecting to Pixhawk...")

# Connect via telem2
# Default telem2 baud = 57600
vehicle = connect('/dev/ttyAMA0', 
                  baud=57600, 
                  wait_ready=True)

print("Connected!")
print("Firmware: ", vehicle.version)
print("Battery:  ", vehicle.battery)
print("GPS:      ", vehicle.gps_0)
print("Mode:     ", vehicle.mode.name)
print("Armed:    ", vehicle.armed)
print("Attitude: ", vehicle.attitude)
print("Location: ", vehicle.location.global_frame)

# Close connection
vehicle.close()
print("Done")
