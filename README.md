# gps_vio
A ROS package: GPS-aided VIO based on Realsense T265 and PX4

## Installation
#### Dependences
1. [catkin_simple]()
2. Realsense SDK: [librealsense](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md)
3. [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
4. [gtsam](https://gtsam.org/get_started/)
#### gps_vio
```
git clone https://github.com/ZhiangChen/gps_vio.git
cd ~\catkin_ws
catkin build gps_vio
```

## Getting Started
#### Gazebo
PX4 sitl has been used in Gazebo to simulate an aircraft with GPS. VIO is replaced by Gaussian-noise-added ground truth from a gazebo plugin, [libgazebo_ros_p3d](http://docs.ros.org/electric/api/gazebo_plugins/html/group__GazeboRosP3D.html). The reason for this simplification is that only local transitions from VIO are used to build a factor graph and the global drift correction dependes on global pose estimation from FCU and GPS.
```
roslaunch px4 mavros_posix_sitl.launch
roslaunch gps_vio gazebo_test.launch
```

#### Real world
We have Pixhawk 2 with an RTK GPS and an Intel NUC as a companion computer. A Realsense T265 tracking camera is connected with the Intel NUC. First, PX4 and Realsense T265 need to be launched on the Intel NUC
```
roslaunch px4 px4.launch
roslaunch realsense2_camera rs_t265.launch
```
We use the default camera configuration. If you want to change it, 
```
rosrun rqt_reconfigure rqt_reconfigure
```
Lastly, launch gps_vio
```
roslaunch gps_vio gps_vio.launch
```

## Factor Graph
