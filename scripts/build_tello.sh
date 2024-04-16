#!/bin/bash

cd ~/tello-slam-and-recognition/workspace #set project root directory
# rm -rf build install log

rosdep install -i --from-path src
colcon build --symlink-install --packages-select tello tello_control tello_msg
