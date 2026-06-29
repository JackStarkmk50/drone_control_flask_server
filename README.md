## Important: DroneKit is older version may properly work in python version <3.10

## TELE21 & TELEM2 ports

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 (red) | VCC | +5V |
| 2 (blk) | TX (OUT) | +3.3V |
| 3 (blk) | RX (IN) | +3.3V |
| 4 (blk) | CTS | +3.3V |
| 5 (blk) | RTS | +3.3V |
| 6 (blk) | GND | GND |


## TELEM2 -> PI 

| TELEM2 Pin | Signal | Pi Pin |
| --- | --- | --- |
| 1 (red) | VCC | 2/4 (if need) |
| 2 (blk) | TX (OUT) | 10 RX (IN) |
| 3 (blk) | RX (IN) | 8 TX (OUT) |
| 4 (blk) | CTS | |
| 5 (blk) | RTS | |
| 6 (blk) | GND | 6/9 GND |

Once connected the Raspberry Pi with Pixhawk, do some below configuration to complete the setup

```
sudo raspi-config

sudo nano /boot/config.txt

sudo reboot

sudo apt-get update

sudo apt-get install python3-pip

sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-matplotlib python3-lxml libxml2-dev libxslt-dev

sudo pip install PyYAML mavproxy

sudo mavproxy.py --master=/dev/ttyAMA0
```

## Do watch the below video for the proper connection and dependencies installation

[
How To Connect PixHawk to Raspberry Pi and NVIDIA Jetson](https://www.youtube.com/watch?v=nIuoCYauW3s)


## Note: During the installation you may stumble upon the errors

- you can't install packages with sudo pip, use python3-xyz for the installation
- pip or pip3 install may not work when installing the packages globally
- use *--break-system-packages* to install the pip packages globally, exanple: pip install PyYAML mavproxy --break-system-packages
- the above attribute lets you install globally bypassing the gate.

## Alternate Sollution

- Create a Virtual Environment and install all the pip packages
- the Virtual Environment lets you install all the packages without any errors.

## For Drone controlling

- Dronekit python packages is used to controll the drone with the functions.
- Smaller LLM will be used to controll the drone and analyse the drone data.

## Successfull connection

- once the connection is successfull, you'll get the below outputs in the terminal 
and you can provide commands in the terminal itself to communicate with pixhawk

```
virtua@dynamics:~ $ mavproxy.py --master=/dev/ttyAMA0
Connect /dev/ttyAMA0 source_system=255
Log Directory:
Telemetry log: mav.tlog
Waiting for heartbeat from /dev/ttyAMA0
MAV> AP: ArduCopter V4.6.3 (92b0cd78)
AP: ChibiOS: 88b84600
AP: Pixhawk1 004F003F 31385113 38353731
AP: IOMCU: 420 1001 411FC231
AP: RCOut: PWM:1-14
AP: IMU0: fast sampling enabled 8.0kHz/1.0kHz
AP: Frame: QUAD/X
Detected vehicle 1:1 on link 0
online system 1
STABILIZE> Mode STABILIZE
AP: ArduCopter V4.6.3 (92b0cd78)
AP: ChibiOS: 88b84600
AP: Pixhawk1 004F003F 31385113 38353731
AP: IOMCU: 420 1001 411FC231
AP: RCOut: PWM:1-14
AP: IMU0: fast sampling enabled 8.0kHz/1.0kHz
AP: Frame: QUAD/X
AP: ArduCopter V4.6.3 (92b0cd78)
AP: ChibiOS: 88b84600
AP: Pixhawk1 004F003F 31385113 38353731
AP: IOMCU: 420 1001 411FC231
AP: RCOut: PWM:1-14
AP: IMU0: fast sampling enabled 8.0kHz/1.0kHz
AP: Frame: QUAD/X
MAV> Received 1001 parameters (ftp)
Saved 1001 parameters to mav.parm
fence present
AP: PreArm: RC not found
```

## Use Ngrok token for the tunneling
```NGROK_AUTH_TOKEN = "3BhG38tEop2npZ0tQMbWpxj6l60_6sWQ9zg35sSiYPyj2tYZE"```

Through the tunneling the raspberry pi can get command from anywhere.
** install Ngrok on the Raspberry pi OS with the below commands **

```
curl -sSL https://ngrok-agent.s3.amazonaws.com/ngrok.asc \
  | sudo tee /etc/apt/trusted.gpg.d/ngrok.asc >/dev/null \
  && echo "deb https://ngrok-agent.s3.amazonaws.com bookworm main" \
  | sudo tee /etc/apt/sources.list.d/ngrok.list \
  && sudo apt update \
  && sudo apt install ngrok
```
Then add the ngrok authtoken you got from the dashboard

``` ngrok authtoken <NGROK_AUTHTOKEN> ``` -> replace the <NGROK_AUTHTOKEN>
with you token

