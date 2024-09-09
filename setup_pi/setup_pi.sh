#!/usr/bin/env bash

set -eux -o pipefail

if [[ $# -ne 3 ]]; then
    echo "Incorrect number of arguments. Usage: 'setup_pi.sh mycobot_23 secret_password'"
    exit 1
fi

WLAN_SSID=$1
WLAN_PASS=$2

sudo apt update

curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh ./get-docker.sh
sudo groupadd docker
sudo usermod -aG docker $USER
sudo docker pull mzandtheraspberrypi/mycobot-ros2:1.0.0

nmcli d wifi hotspot ifname wlan0 ssid ${WLAN_SSID} password ${WLAN_PASS} con-name my-hotspot
nmcli c down my-hotspot
nmcli connection modify my-hotspot connection.autoconnect yes
nmcli c up my-hotspot