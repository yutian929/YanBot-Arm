#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

sudo apt install udev
check_success
sudo apt install can-utils
check_success
# realsense2 libs
sudo apt-get install ros-${ROS_DISTRO}-image-transport
check_success
sudo apt install ros-${ROS_DISTRO}-rgbd-launch
check_success
sudo apt-get install ros-${ROS_DISTRO}-ddynamic-reconfigure
check_success
