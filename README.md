# mycobot_communications


This is a repo to implement control of the mycobot robotic arm.

## Setup

First plugin the robot to power. Plug a mouse and keyboard into it. Plug a monitor into it. Plug the gripper into the arm (stats on gripper [here](https://docs.elephantrobotics.com/docs/gitbook-en/2-serialproduct/2.7-accessories/2.7.3%20grip/2.7.3.1-ag.html)). Start the robot and connect to wifi using the Ubuntu Desktop Wifi manager.

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
