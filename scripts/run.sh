#!/bin/bash

# Source the workspace
d ~/tello-slam-and-recognition/workspace #set project root directory
. install/setup.bash

# Run packages
cd src
ros2 launch launch.py