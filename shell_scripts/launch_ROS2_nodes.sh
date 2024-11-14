#!/bin/bash

#move to root workspace folder 
cd ~/ROS-Wheel-Castor-Bot

#source ros
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc


# probraly can create pkg ahead of time and push to repo to pull later
# cd ~/ros2_ws/src
# ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>

cd ~/ros2_ws
colcon build
#build packages