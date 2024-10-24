#!/usr/bin/env bash

set -eux -o pipefail

mkdir -p /home/er/ros_ws/src
git clone https://github.com/VModugno/mycobot_client /home/er/ros_ws/src

sudo docker pull mzandtheraspberrypi/mycobot-ros2:1.2.0
echo "alias mycobot-up='docker run -it --network host --device /dev/ttyAMA0 --volume /home/er/:/mnt_folder -v /dev:/dev --device-cgroup-rule \"c 81:* rmw\"  --device-cgroup-rule \"c 189:* rmw\" --rm mzandtheraspberrypi/mycobot-ros2:1.2.0'" >>/home/er/.bashrc
echo "alias mycobot-client-up='xhost +local:root; sudo docker run -it --network host --env=DISPLAY --env=QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --volume=/home/er/ros_ws/:/ros_ws --volume /home/er/img_plots:/root/img_plots --workdir /ros_ws mzandtheraspberrypi/mycobot-client-humble:1.0.0; xhost -local:root;'" >>/home/er/.bashrc
sudo journalctl --vacuum-time=10d # trying to save some disk space
sudo reboot