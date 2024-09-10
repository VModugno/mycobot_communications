#!/usr/bin/env bash

set -eux -o pipefail

if [[ $# -ne 3 ]]; then
    echo "Incorrect number of arguments. "
    echo $#
    echo " Usage: 'setup_pi.sh mycobot_23 secret_password'"
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

# https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nm-settings-nmcli.html
# useful options. We want to setup a wifi network and let folks connect to it and get an ip address
# if someone is using a VM and bridged network setting, the VM has same MAC address
# we still want to let the VM get an ip address
nmcli d wifi hotspot ifname wlan0 ssid ${WLAN_SSID} password ${WLAN_PASS} con-name my-hotspot
nmcli c down my-hotspot
nmcli c modify my-hotspot connection.autoconnect yes
nmcli c modify my-hotspot connection.autoconnect-priority 999

nmcli c up my-hotspot