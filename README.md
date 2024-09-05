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

# pymycobot version
by default on the pi's we have pymycobot==2.7.5 installed.
this is quite old.

```
pymycobot_version '2.7.5'
average timings
{'get_angles': 0.10741803957068402, 'get_encoders': 0.10487154255742612}
number of calls
{'get_angles': 23, 'get_encoders': 23}
```

```
python_version '2.7.17 (default, Feb 27 2021, 15:10:58) \n[GCC 7.5.0]'
pymycobot_version '2.7.5'
average timings
{'get_angles': 0.1082292530271742, 'get_encoders': 0.1065657999780443}
number of calls
{'get_angles': 36, 'get_encoders': 36}
```

```
python_version '3.8.10 (default, Jul 29 2024, 17:02:10) \n[GCC 9.4.0]'
pymycobot_version '3.4.9'
robot_version: 1
sys_version: 3.1
robo_id: 0
basic_firmware_version: None
atom_version: None
average timings
{'get_angles': 0.009076217752048346, 'get_encoders': 0.008493457721475282}
number of calls
{'get_angles': 341, 'get_encoders': 341}
```


```
sudo docker build -t mycobot-noetic -f Dockerfile .
sudo docker run -it --network host --device /dev/ttyAMA0 --volume /home/ubuntu/catkin_ws/src/:/mnt_folder mycobot-noetic
source devel/setup.bash
roslaunch mycobot_interface communication_topic.launch
```

Atom version we will update with my studio. Use the my studio on the pi. The mystudio that we can download from the current website doesn't seem to have atom version available, just M5. Plug the USB C cable into the top atom port, and then into the pi. This is launchable from the desktop of the pi.

On linux, make it executable and run it. Select mycobot 280, mycobot 280 for Pi, and then login.

```
https://docs.elephantrobotics.com/docs/gitbook-en/4-BasicApplication/4.1-myStudio/
PI	RaspberryPI 4B	ubuntu	v18.04.is recommened
Atom	atomMain	v4.1 is recommended for robots labelled ER28001202200415 and before，or not lablled; v5.1 is recommended for robots lablled ER28001202200416 and after
```