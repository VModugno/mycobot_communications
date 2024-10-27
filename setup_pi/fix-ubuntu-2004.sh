#!/usr/bin/env bash

set -eux -o pipefail

mkdir -p /home/er/ros_ws/src
git clone https://github.com/VModugno/mycobot_client /home/er/ros_ws/src/mycobot_client

sudo docker pull mzandtheraspberrypi/mycobot-client-humble:1.2.0
echo "alias mycobot-client-up='docker run -it --network host --volume /dev/shm:/dev/shm --volume=/home/er/ros_ws/:/ros_ws --volume /home/er/img_plots:/root/img_plots --workdir /ros_ws --rm mzandtheraspberrypi/mycobot-client-humble:1.2.0'" >>/home/er/.bashrc
sed -i 's/.*alias mycobot-up.*/d' /home/er/.bashrc
echo "alias mycobot-up='docker run -it --network host --volume /dev/shm:/dev/shm --device /dev/ttyAMA0 --volume /home/er/:/mnt_folder -v /dev:/dev --device-cgroup-rule \"c 81:* rmw\"  --device-cgroup-rule \"c 189:* rmw\" --rm mzandtheraspberrypi/mycobot-ros2:1.1.0'" >>/home/er/.bashrc