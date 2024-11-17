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

echo "Updating and installing locales"
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-$rosDistro-desktop
sudo apt install ros-$rosDistro-ros-base
sudo apt install ros-dev-tools

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