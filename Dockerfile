FROM ros:noetic-ros-core

RUN sudo apt-get update
RUN sudo apt-get install build-essential cmake git nano python3-pip -y

RUN pip install mycobotclient
RUN mkdir -p /home/ubuntu/catkin_ws/src
WORKDIR /home/ubuntu/catkin_ws/src
RUN git clone https://github.com/VModugno/mycobot_communications.git
WORKDIR /home/ubuntu/catkin_ws
RUN catkin_make
RUN source devel/setup.bash
