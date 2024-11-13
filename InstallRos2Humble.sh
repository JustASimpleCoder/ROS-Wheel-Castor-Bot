#!/bin/bash

# List of valid ROS 2 distributions
valid_distros=("ardent" "bouncy" "crystal" "dashing" "eloquent" "foxy" "galactic" "rolling" "humble" "iron")

# Check if running as root otherwise exit script
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (e.g., 'sudo bash install_ros2.sh <distro_name>')"
    exit
fi

# Ensure a distro argument is provided
if [ -z "$1" ]; then
    echo "Usage: sudo bash install_ros2.sh <rosDistro>"
    exit
fi

# Convert distro name to lowercase
rosDistro=$(echo "$1" | tr '[:upper:]' '[:lower:]')

# Check if entered distro is valid
if [[ ! " ${valid_distros[@]} " =~ " ${rosDistro} " ]]; then
    echo "Invalid ROS 2 distribution: ${rosDistro}"
    echo "Valid options are: ${valid_distros[@]}"
    exit
fi

echo "Installing ROS 2 ${rosDistro^}..."

# Check Ubuntu version (20.04 or later)
. /etc/os-release
if [[ "$VERSION_ID" < "20.04" ]]; then
    echo "This script requires Ubuntu 20.04 or later."
    exit
fi

# Update and upgrade system
apt update && apt upgrade -y

# Add ROS 2 repository
apt install -y software-properties-common
add-apt-repository universe
apt update && apt install -y curl gnupg lsb-release

# Add ROS 2 GPG key
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -

# Add ROS 2 apt repository
sh -c "echo 'deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main' > /etc/apt/sources.list.d/ros2-latest.list"

# Update package index
apt update

# Install ROS 2 base and developer packages
apt install -y "ros-$rosDistro-desktop"

# Source ROS 2 setup in the current shell
echo "source /opt/ros/$rosDistro/setup.bash" >> ~/.bashrc
source /opt/ros/$rosDistro/setup.bash

# Install dependencies for building packages
apt install -y python3-rosdep python3-colcon-common-extensions

# Initialize rosdep
rosdep init
rosdep update

# Notify user of successful installation
echo "ROS 2 ${rosDistro^} has been installed and configured!"
echo "To start using ROS 2, open a new terminal or run 'source /opt/ros/$rosDistro/setup.bash' in the current one."
