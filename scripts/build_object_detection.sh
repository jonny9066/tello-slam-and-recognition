cd ~/tello-slam-and-recognition/workspace #set project root directory
# rm -rf build install log

# pwd
rosdep install -i --from-path src
colcon build --symlink-install --packages-select object_detection
