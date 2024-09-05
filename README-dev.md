# dev README

Notes for developpers...

# pymycobot version
by default on the pi's with Ubuntu 18 we have pymycobot==2.7.5 installed and python 2.
this is quite old and slow.

```
python_version '2.7.17 (default, Feb 27 2021, 15:10:58) \n[GCC 7.5.0]'
pymycobot_version '2.7.5'
average timings
{'get_angles': 0.1082292530271742, 'get_encoders': 0.1065657999780443}
number of calls
{'get_angles': 36, 'get_encoders': 36}
```

If we use python3 and a newer version of pymycobot (this is the criticial bit) we get way better performance.
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

# notes on docker

Some of us use docker. You can use the Dockerfile here for ros-noetic.

```
sudo docker build -t mycobot-noetic -f Dockerfile .
sudo docker run -it --network host --device /dev/ttyAMA0 --volume /home/ubuntu/catkin_ws/src/:/mnt_folder mycobot-noetic
source devel/setup.bash
roslaunch mycobot_interface communication_topic.launch
```

# notes on firmware upgrades
Atom version we will update with my studio. Use the my studio on the pi. The mystudio that we can download from the current website doesn't seem to have atom version available, just M5. Plug the USB C cable into the top atom port, and then into the pi. This is launchable from the desktop of the pi.

```
https://docs.elephantrobotics.com/docs/gitbook-en/4-BasicApplication/4.1-myStudio/
PI	RaspberryPI 4B	ubuntu	v18.04.is recommened
Atom	atomMain	v4.1 is recommended for robots labelled ER28001202200415 and before，or not lablled; v5.1 is recommended for robots lablled ER28001202200416 and after
```