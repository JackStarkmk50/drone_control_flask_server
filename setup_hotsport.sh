# Save as setup_hotspot.sh
# Run ONCE to configure Pi as AP

#!/bin/bash

echo "Installing required packages..."
sudo apt-get update
sudo apt-get install -y \
    hostapd \
    dnsmasq \
    dhcpcd5

echo "Stopping services..."
sudo systemctl stop hostapd
sudo systemctl stop dnsmasq

# Configure static IP for Pi
sudo tee /etc/dhcpcd.conf > /dev/null << EOF
interface wlan0
    static ip_address=192.168.4.1/24
    nohook wpa_supplicant
EOF

# Configure DHCP server
sudo tee /etc/dnsmasq.conf > /dev/null << EOF
interface=wlan0
dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h
domain=local
address=/drone.local/192.168.4.1
EOF

# Configure Access Point
sudo tee /etc/hostapd/hostapd.conf > /dev/null << EOF
interface=wlan0
driver=nl80211
ssid=DroneControl
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=drone1234
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
EOF

sudo tee /etc/default/hostapd > /dev/null << EOF
DAEMON_CONF="/etc/hostapd/hostapd.conf"
EOF

echo "Enabling services..."
sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo systemctl enable dnsmasq

echo "Done! Reboot to activate hotspot"
echo "SSID: DroneControl"
echo "Password: drone1234"
echo "Pi IP: 192.168.4.1"
