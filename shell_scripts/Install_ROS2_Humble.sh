#!/bin/bash


# Check if running as root otherwise exit script
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (e.g., 'sudo bash install_ros2.sh <distro_name>')"
    exit
fi



# List of valid ROS 2 distributions
valid_distros=("ardent" "bouncy" "crystal" "dashing" "eloquent" "foxy" "galactic" "rolling" "humble" "iron")

# Convert distro name to lowercase
rosDistro=$(echo "$1" | tr '[:upper:]' '[:lower:]')
# Ensure a distro argument is provided, otherwise set default to humble

if [ -z "$1" ]; then
    echo "No ROS2 distibution given as a parameter, setting default to humble"
    rosDistro="${1:-humble}"
else
    if [[ ! " ${valid_distros[@]} " =~ " ${rosDistro} " ]]; then
        echo "Invalid ROS 2 distribution: ${rosDistro}"
        echo "Valid options are: ${valid_distros[@]}"
        exit
    fi
fi

# Check if entered distro is valid


echo "Installing ROS 2 ${rosDistro^}..."

# Check Ubuntu version (20.04 or later)
. /etc/os-release
if [[ "$VERSION_ID" < "20.04" ]]; then
    echo "This script requires Ubuntu 20.04 or later."
    exit
fi

# Update and upgrade system
sudo apt update && sudo  apt upgrade -y

# Add ROS 2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -

# Add ROS 2 apt repository
sudo sh -c "echo 'deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main' > /etc/apt/sources.list.d/ros2-latest.list"

# Update package index
sudo apt update

# Install ROS 2 base and developer packages
sudo apt install -y "ros-$rosDistro-desktop"

# Source ROS 2 setup in the current shell
sudo echo "source /opt/ros/$rosDistro/setup.bash" >> ~/.bashrc
sudo source /opt/ros/$rosDistro/setup.bash

# Install dependencies for building packages
sudo apt install -y python3-rosdep python3-colcon-common-extensions

# Initialize rosdep
sudo rosdep init
sudo rosdep update

# Notify user of successful installation
sudo echo "ROS 2 ${rosDistro^} has been installed and configured!"
sudo echo "To start using ROS 2, open a new terminal or run 'source /opt/ros/$rosDistro/setup.bash' in the current one."
