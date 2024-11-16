#!/bin/bash

#move to root workspace folder 
cd ~/ROS-Wheel-Castor-Bot

#source ros
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc


# probraly can create pkg ahead of time and push to repo to pull later
# cd ~/ros2_ws/src
# ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>


#build packages
cd ~/ros2_ws

colcon build # if desired single package only colcon build --packages-select my_package
source install/local_setup.bash

declare -a packageNames =("subpub_test","miroros_node")
for i in "${arr[@]}"
do
   ros2 run packageNames i
done


ros2 run my_package my_node
