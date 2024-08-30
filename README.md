# mycobot_communications


This is a repo to implement control of the mycobot robotic arm.

## Setup

First plugin the robot to power. Plug a mouse and keyboard into it. Plug a monitor into it. Start the robot and connect to wifi using the Ubuntu Desktop Wifi manager.

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

From here, we can run rosmaster and our controller code, and try publishing a msg to control the arm.
In one terminal:
```
roscore
```
In another terminal bring up the arm controller:

```
roslaunch mycobot_interface commmunication_service.launch port:=/dev/ttyAMA0
```

