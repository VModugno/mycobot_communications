# setup_pi
If you are configuring many of the robot arms, you will need to setup our configurations. This includes broadcasting a wifi network automatically (setting up the raspberry pi in AP mode), installing docker, and pulling our docker image.

You can copy/paste or clone this repo, and run it with sudo. It takes the name of the wifi network, which we name after the robot number, and the password. Ask someone for the password we use.
```
sudo bash setup_pi.sh mycobot_23 my_wifi_pass
```

# testing
After we run the setup script, we should restart the pi to apply some of the changes (specifically user group rules). Then:

```
mycobot-up
source install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch mycobot_interface_2 mycobot_comms_launch.py use_realsense:=True
```

From here on your client computer connect to the robot's wifi network, then test that the inverse kinematics demo works and that in rviz2 you can see the camera.
```
ros2 run mycobot_client_2 cobot_ik_demo
```

Add camera topic through gui, select compressed color camera. Then edit the frame to be camera_link. Then ensure you see an image.
```
rviz2
```