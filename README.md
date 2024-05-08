
# Object mapping in 3D environment using SLAM

## Overview
Our goal is to create a 3D map of the environment in correct scale and to accurately place objects in it.

![Output sample](https://github.com/jonny9066/tello-slam-and-recognition/blob/main/readme/objectdet2.gif?raw=true)   
(In blue is the point cloud associated with the object currently in focus, white is the detection, and the red arrow is the camera pose.)

## Background
The algorithms we use are the YOLO neural network and ORB-SLAM. 

todo remove
We use YOLO to compute bounding boxes around objects in the 2D image, and ORB-SLAM to compute a 3D map of the world.

We detect objects in the 2D image using the YOLO model, and then use our algorithm to detect the corresponding objects in the ORB-SLAM generated 3D point cloud. We proceed to present informal descriptions of the algorithms that are relevant to out work.

### ORB-SLAM
A fast algorithm for SLAM that uses ORB features. Initially, a 3D scene is constructed from a pair of frames by matching feature points, computing the essential matrix, and then triangulating the points. The 3D scene consists of points and of keyframes, where keyframes are the poses of "useful" frames. As a new frame is processed, its features are matched against previous frames, as well as against nearby frames, and only its position is computed. If a frame has enough "useful" features, then it is added to the map, and new points are triangulated.

To reduce error and keep the map consistent, bundle adjestemnt is performed with each new frame, and with the addition of each frame to the map (albeit differently in each case). This minimizes the reprojection error of the points to the keyframes and makes the map more consistent. Additionally, keyframes and points may be removed to keep the map smaller, which allows the algorithm to be more effcient. Finally, loop closure is computed when two distant frames match, which is checked with the addition of each new frame. When a loop is detected, then a transformation is computed between the two ends, and frames and points on both ends of the loop are updated with this transformation.  



### YOLO Object Detection
A neural network that receives an image and outputs a bounding box around each detected object together with scores for different object categories. Instead of performing the detection of bounding boxes in one network and object detection in another network, both tasks are performed in one network, resulting in very good performance. 


## Our Algorithm
ORB-SLAM provides a 3D map that consists of a point cloud corresponding to ORB features detected in images, and YOLO provides bounding boxes around objects in the 2D image. Our challenge is to determine which points in the cloud correspond to the object, to determine its location from it, and to update the location as the map is updated by ORB-SLAM.

The outline of our algorithm is as follows. The first thing we do is to compute the scale of the scene. Then we enter object detection mode, where with each new YOLO detection we either add a new object to the map or update the location of an existing object that is re-detected.

### Scale
To compute the scale, we perform a measurement of an object in the 3D world and the same measurement in the real world. Then we compute the scale $s$ using the equation $s\cdot d_m = d_r$, where $d_m$ is the 3D world distance and $d_r$ is the real-world distance.  
In case the object is a chair, we use the distance between its two legs, computed as follows:
1. Several keypoings ear each leg are found in the 2D image.
2. Corresponding 3D map points are computed.
3. The distance is computed between the two points.

![chair scale](https://github.com/jonny9066/tello-slam-and-recognition/blob/main/readme/scale_line.png?raw=true)  
(The red line corresponds to distance measureemnt in the 3D world)

### Object Detection
We focus on one detected object at a time when there are several in the image. We first compute an estimate of the current object. We repeat this several times for consistency. When we have an estimate, we check whether it collides with an already detected object. If so, we assume it is the same object, and update its location. Otherwise, we add a new object coordinate to the list of detected objects. 

Considerations
1. An object is represented as the feature points that it was detected with. This allows its location to be updated by ORB-SLAM during bundle adjustemnt and loop closure.
2. There is drift due to the nature of the ORB-SLAM algorithm. Updating the location of an object with each detection prevents it from being detected twice after the first decection drifts.

The objects are mapped as follows 
![2dmap](https://github.com/jonny9066/tello-slam-and-recognition/blob/main/readme/2dmap.png?raw=true)  
(blue line is the camera pose nad the numbers represent the order of detection)  
The real scene for reference, where the chairs are the detected objects:
![chair_scene](https://github.com/jonny9066/tello-slam-and-recognition/blob/main/readme/scene_chairs.jpg?raw=true) 

Distances between objects as measured by our algorithm are:  
1 and 2: 172cm  
2 and 3: 133cm  
3 and 4: 164cm  
while the distances measured in the real world are:  
1 and 2: 170cm  
2 and 3: 140cm  
3 and 4: 180cm  


## Setup
Tools used are: C++, Python, ROS, Docker.  

We provide instructions that were tested on Ubuntu 22. The code is split over two repositories. The first repository is this one, and it uses ROS2. The scond repository uses ROS1 and must be run from inside a container due to incompatibility of ROS1 with Ubuntu 22. Addionally, the container allows easy building of ORB-SLAM 2, which may cause difficulties with newer versions of Ubuntu and required libraries.

### Host
Setup environment
1. Install ROS2 as described in https://docs.ros.org/en/humble/Installation.html.
2. Setup ROS1-bridge as described in https://jaypr.notion.site/ROS2-Humble-ROS1-Noetic-Bridge-Tutorial-using-ros1_bridge-c158e43e755c440e9dd378288df1e3d6, but with the following modifications. (1) rename noetic to melodic in the dockerfile and set the tutorials flag to 0, (2) can ignore rocker and use with regular docker.

Build project
1. Pull this repository. 
2. (optional) Mark `scripts` as exscutable with `chmod +x`.
3. Install dependencies using `scripts/install_dependencies.sh`.
4. Execute the script `scripts/build_all.sh`.

Run using `scripts/run.sh`.

Run ROS1 Bridge with:
```
source /opt/ros/humble/setup.bash
source ~/ros-humble-ros1-bridge/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge
```


### Container
Setup container  
1. Install docker https://www.docker.com/get-started/.
2. Setup the following container https://github.com/HyeonJaeGil/orbslam2-ros. Should be able ro run ORB-SLAM 2 (with no input).
3. Run  `apt-get install python-pip` and then `pip install "pybind11[global]"`.


Setup our code inside the container
1. Pull the repository https://github.com/jonny9066/object_mapping_ROS_ORB_SLAM. 
2. Do not follow all instructions there, as many dependencies are already installed. Follow only the instructions for h264decoder and for building ORB-SLAM 2.

Run
1. Execute the command `xhost + local:docker` on host machine.  
2. Inside docker, run
```
cd ~/ROS/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws
roslaunch flock_driver orbslam2_with_cloud_map.launch
```


## Acknowledgements
We used code from https://github.com/tentone/tello-ros2 and 
https://github.com/tau-adl/Tello_ROS_ORBSLAM.


## Future directions
1. Use static objects to improve ORB-SLAM tracking, and ignore moving objects.
2. Navigate from object to object using a programmable drone.