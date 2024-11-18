#!/bin/bash

# Define variables
ROS_DISTRO="humble"

# Install ROS 2 Humble
echo "Installing ROS 2 Humble..."
sudo apt update && sudo apt install -y ros-${ROS_DISTRO}-desktop
source /opt/ros/${ROS_DISTRO}/setup.bash

# Create a Docker container for optional use (comment if not needed)
# Uncomment the next line if you want to use Docker
# docker run -it --net=host -v /dev:/dev --privileged ros:${ROS_DISTRO}

# Create micro-ROS workspace
echo "Setting up micro-ROS workspace..."
mkdir -p ~/microros_ws/src
cd ~/microros_ws
git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies and install pip
echo "Installing dependencies..."
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
sudo apt-get install -y python3-pip

# Build micro-ROS setup tools
echo "Building micro-ROS tools..."
colcon build
source install/local_setup.bash

# Create firmware workspace
echo "Creating firmware workspace..."
ros2 run micro_ros_setup create_firmware_ws.sh host

# Build firmware
echo "Building firmware..."
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash

# Create micro-ROS agent workspace
echo "Setting up micro-ROS agent workspace..."
ros2 run micro_ros_setup create_agent_ws.sh

# Build micro-ROS agent
echo "Building micro-ROS agent..."
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

# Run micro-ROS agent
echo "Starting micro-ROS agent on UDP port 8888..."
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &

# Run the Ping Pong micro-ROS node
echo "Starting Ping Pong micro-ROS node..."
export RMW_IMPLEMENTATION=rmw_microxrcedds
ros2 run micro_ros_demos_rclc ping_pong &

# Testing Ping Pong Node
echo "Testing the Ping Pong Node..."
# Subscribe to ping topic
gnome-terminal -- bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; ros2 topic echo /microROS/ping"
# Publish a fake ping
gnome-terminal -- bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: \"fake_ping\"}'"
# Subscribe to pong topic
gnome-terminal -- bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; ros2 topic echo /microROS/pong"

# Multinode setup (optional)
echo "To run multiple Ping Pong nodes, open additional terminals and run the following commands:"
echo "cd ~/microros_ws"
echo "source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "source install/local_setup.bash"
echo "export RMW_IMPLEMENTATION=rmw_microxrcedds"
echo "ros2 run micro_ros_demos_rclc ping_pong"

echo "Micro-ROS setup complete."
