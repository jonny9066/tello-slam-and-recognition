#!/bin/bash

cd ~/tello-slam-and-recognition/workspace #set project root directory

rosdep install -i --from-path src
colcon build --symlink-install