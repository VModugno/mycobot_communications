#!/usr/bin/env bash

set -eux -o pipefail

mkdir -p /home/er/ros_ws/src
git clone https://github.com/VModugno/mycobot_client /home/er/ros_ws/src

sudo docker pull mzandtheraspberrypi/mycobot-client-humble:1.2.0
echo "alias mycobot-client-up='docker run -it --network host --volume=/home/er/ros_ws/:/ros_ws --volume /home/er/img_plots:/root/img_plots --workdir /ros_ws --rm mzandtheraspberrypi/mycobot-client-humble:1.2.0'" >>/home/er/.bashrc
sudo journalctl --vacuum-time=10d # trying to save some disk space
sudo reboot