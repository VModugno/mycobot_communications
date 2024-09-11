# mycobot_communications

This is a repo to implement control of the MyCobot 280 and specifically the version with the raspberry pi. The code in this repo is intended to run on the pi and provide an interface to command the robot using ros to other devices on the same network. In another repository, [here](https://github.com/VModugno/mycobot_client), there is example code that runs on network devices to communicate using ros.

## Setup

First plugin the robot to power. Plug a mouse and keyboard into it. Plug a monitor into it. Plug the gripper into the arm, there is a three prong slot near the top of the arm on a side adjacent to the side with the USB C connector (stats on gripper [here](https://docs.elephantrobotics.com/docs/gitbook-en/2-serialproduct/2.7-accessories/2.7.3%20grip/2.7.3.1-ag.html)). The gripper has a pretty short cable by default, you may need to use one of the extension cables provided to give it more leeway to rotate. Start the robot and connect to wifi using the Ubuntu Desktop Wifi manager.

Pick ROS 1 or ROS 2. We recommend ROS 2. If you use ROS1 checkout the branch `ros-noetic-1.0.0` in this repo. Otherwise use `main` for ROS2.

Note, you can optionally upgrade the ubuntu image on the arm. But, we haven't yet found this nescessary. Elephant robotic provides a system image [here](https://www.elephantrobotics.com/en/downloads/) for the mycobot 280 arm. I used the raspberry pi imager tool to flash it to the sd card--importantly I selected to clear the settings the raspberry pi imager has like user/password. I also didn't need to unzip .the .img file to flash it.

### Ros 1

Then, clone the repo into the catkin workspace on the robot. And build the ros workspace. We will modify our `.bashrc` so that each time we open a terminal we can access the packages here.
```
cd /home/ubuntu/catkin_ws/src
git config --global http.sslVerify false
git clone https://github.com/VModugno/mycobot_communications.git
cd /hom/ubuntu/catkin_ws
catkin_make
echo "source /home/ubuntu/catkin_ws/devel/setup.bash">>/home/ubuntu/catkin_ws/.bashrc
source /home/ubuntu/catkin_ws/devel/setup.bash
```

The PI also has an issue where the `ROS_MASTER_URI` and `ROS_IP` variables are set to localhost in the `.bashrc` and this makes it so we can't talk from a computer with ROS Noetic installed to the PI over the network. If you put the IP Address of the PI into the `.bashrc` then it works.

From here, we can run rosmaster and our controller code, and try publishing a msg to control the arm.
In one terminal bring up the arm controller:
```
roslaunch mycobot_interface communication_topic.launch port:=/dev/ttyAMA0
```
In another terminal try to control it by publishing a ros msg:
```
rostopic pub -1 /mycobot/angles_goal mycobot_communication/MycobotSetAngles "{joint_1: -50, speed:30}"
```

### Ros 2
For this we will go into a docker container. We do this because the Ubuntu version running on the raspberry pi is quite old the ROS2 distro we want to use (humble) doesn't support that old Ubuntu. If you are in a raspberry pi prepared by the lab, it will have docker. If not, install it onto the pi using [this](https://docs.docker.com/engine/install/ubuntu/). You shouldn't need to build any docker image, you can simply pull it from docker hub with the below command.

```
sudo docker run -it --network host --device /dev/ttyAMA0 --volume /home/ubuntu/:/mnt_folder --rm mzandtheraspberrypi/mycobot-ros2:1.0.0
source install/setup.bash
export ROS_DOMAIN_ID=10
ros2 run mycobot_interface_2 cobot_comms
```

If you want to echo topics and such, it will be easiest to do from the same docker container in a different bash terminal. You can find the container id by opening a new terminal and running `docker ps` and then in that terminal running `docker exec -it 10a72b2d02fc /bin/bash`, replacing `10a72b2d02fc` with your container id. This will put you inside of the same container, but in a new bash terminal where you can load the packages `source install/setup.bash` and then run ros2 introspection commands.

You can also try moving the robot with something like:
```
ros2 topic pub /mycobot/angles_goal mycobot_msgs_2/msg/MycobotSetAngles "{joint_6: 30, speed: 30}" --once
```

## Troubleshooting
If you can ping the pi, for example the below shows packets sent and received.
```
ping 10.42.0.1
```
But the pi can't ping your computer, look to your firewall in windows. Try turning it off temporarily.