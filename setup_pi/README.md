# setup_pi
If you are configuring many of the robot arms, you will need to setup our configurations. This includes broadcasting a wifi network automatically (setting up the raspberry pi in AP mode), installing docker, and pulling our docker image.

You can copy/paste or clone this repo, and run it with sudo. It takes the name of the wifi network, which we name after the robot number, and the password. Ask someone for the password we use.
```
sudo bash setup_pi.sh mycobot_23 my_wifi_pass
```