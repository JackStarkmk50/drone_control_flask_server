#!/bin/bash
# wifi_manager.sh
# Copy to: /home/pi/
# Runs on boot via wifi-manager.service
# Tries known WiFi networks for 20s, then falls back to hotspot "DroneControl"
#
# One-time hotspot profile setup (run once manually on Pi):
#   sudo nmcli connection add \
#     type wifi ifname wlan0 con-name DroneControl autoconnect no \
#     ssid DroneControl mode ap \
#     ipv4.method shared ipv4.addresses 192.168.4.1/24 \
#     wifi-sec.key-mgmt wpa-psk wifi-sec.psk "drone1234"

HOTSPOT_CON="DroneAP"
WAIT_SECONDS=20

echo "[wifi_manager] Waiting up to ${WAIT_SECONDS}s for WiFi connection..."

for i in $(seq 1 $WAIT_SECONDS); do
    STATE=$(nmcli -t -f STATE general 2>/dev/null)
    if echo "$STATE" | grep -q "connected"; then
        IP=$(nmcli -t -f IP4.ADDRESS device show wlan0 2>/dev/null | cut -d: -f2 | cut -d/ -f1)
        echo "[wifi_manager] WiFi connected — IP: $IP"
        exit 0
    fi
    sleep 1
done

echo "[wifi_manager] No WiFi found — activating hotspot '$HOTSPOT_CON'"
nmcli connection up "$HOTSPOT_CON"

if [ $? -eq 0 ]; then
    echo "[wifi_manager] Hotspot active — connect to '$HOTSPOT_CON', IP: 192.168.4.1"
else
    echo "[wifi_manager] ERROR: Failed to start hotspot. Check nmcli connection list."
    exit 1
fi
