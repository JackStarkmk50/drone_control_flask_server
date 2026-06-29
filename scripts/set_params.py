from dronekit import connect
import time

print("Connecting...")
vehicle = connect('/dev/ttyAMA0',
                  baud=57600,
                  wait_ready=True)

print("Connected!")

# Read a parameter
arming_check = vehicle.parameters['ARMING_CHECK']
print(f"Current ARMING_CHECK: {arming_check}")

# Change a parameter
vehicle.parameters['ARMING_CHECK'] = 0
time.sleep(1)

# Verify change
new_val = vehicle.parameters['ARMING_CHECK']
print(f"New ARMING_CHECK: {new_val}")

# Change multiple params at once
params_to_set = {
    'ARMING_CHECK': 0,
    'SERIAL2_BAUD': 57,
    'SERIAL2_PROTOCOL': 2,
    'FS_EKF_ACTION': 2,
    'LOIT_SPEED': 500,
}

for param, value in params_to_set.items():
    vehicle.parameters[param] = value
    print(f"Set {param} = {value}")
    time.sleep(0.1)

print("All params set!")
vehicle.close()
