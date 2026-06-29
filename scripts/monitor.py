from dronekit import connect
import time

vehicle = connect('/dev/ttyAMA0',
                  baud=57600,
                  wait_ready=True)

print("Monitoring... Ctrl+C to stop\n")

try:
    while True:
        print(f"\r"
              f"Mode: {vehicle.mode.name:12}"
              f"Armed: {str(vehicle.armed):6}"
              f"Alt: {vehicle.location.global_relative_frame.alt:5.1f}m  "
              f"Bat: {vehicle.battery.voltage:4.1f}V  "
              f"Sats: {vehicle.gps_0.satellites_visible:2}  "
	      f"HDOP: {vehicle.gps_0.eph} "
              f"Pitch: {vehicle.attitude.pitch:6.3f}  "
              f"Roll: {vehicle.attitude.roll:6.3f} "
	      f"YAW: {vehicle.attitude.yaw:6.3f} "
	      f"Airspeed: {vehicle.airspeed:.1f} m/s",
              end='', flush=True)
        time.sleep(0.1)

except KeyboardInterrupt:
    vehicle.close()
    print("\nDone")
