#!/usr/bin/env bash

set -eux -o pipefail

if [[ $# -le 1 ]]; then
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
sudo usermod -aG docker $USER
sudo docker pull mzandtheraspberrypi/mycobot-ros2:1.1.0

# https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nm-settings-nmcli.html
# useful options. We want to setup a wifi network and let folks connect to it and get an ip address
nmcli d wifi hotspot ifname wlan0 ssid ${WLAN_SSID} password ${WLAN_PASS} con-name my-hotspot
nmcli c down my-hotspot
nmcli c modify my-hotspot connection.autoconnect yes
nmcli c modify my-hotspot connection.autoconnect-priority 999

nmcli c up my-hotspot

mkdir -p /home/er/ros_ws/src
git clone https://github.com/VModugno/mycobot_client /home/er/ros_ws/src

sudo docker pull mzandtheraspberrypi/mycobot-client-humble:1.2.0
echo "alias mycobot-up='docker run -it --network host --device /dev/ttyAMA0 --volume /home/er/:/mnt_folder -v /dev:/dev --device-cgroup-rule \"c 81:* rmw\"  --device-cgroup-rule \"c 189:* rmw\" --rm mzandtheraspberrypi/mycobot-ros2:1.1.0'" >>/home/er/.bashrc
echo "alias mycobot-client-up='docker run -it --network host --volume=/home/er/ros_ws/:/ros_ws --volume /home/er/img_plots:/root/img_plots --workdir /ros_ws mzandtheraspberrypi/mycobot-client-humble:1.2.0'" >>/home/er/.bashrc
sudo journalctl --vacuum-time=10d # trying to save some disk space
sudo reboot