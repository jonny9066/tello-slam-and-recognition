#!/bin/bash

if (( $EUID > 0 )); then
	echo " - Please run as root"
	exit
fi


# Install project dependencies
echo " - Python dependencies"
pip3 install catkin_pkg rospkg av image opencv-python djitellopy2 pyyaml empy==3.3.4 lark
apt install python3-tf*

echo " - CPP dependencies"
apt install ros-humble-ament-cmake* ros-humble-tf2* ros-humble-rclcpp* ros-humble-rosgraph*

echo " - Rviz and RQT Tools"
apt install ros-humble-rviz* ros-humble-rqt*
