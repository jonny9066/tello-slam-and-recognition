#!/bin/bash

# Source the workspace
cd ~/tello-slam-and-recognition/workspace #set project root directory

../scripts/build_all.sh
source install/setup.bash

# Run packages
cd src
ros2 launch launch.py