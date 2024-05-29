#!/bin/bash

# CHANGE LINE BELOW FOR UPDATING THE PACKAGE NAME
export PACKAGE_NAME='usb_cam'

# Function to print in red
print_red() {
    echo -e "\033[0;31m$1\033[0m"
}

# Function to print in blue
print_blue() {
    echo -e "\033[0;34m$1\033[0m"
}

source /opt/ros/${ROS_DISTRO}/setup.bash

print_blue "BUILDING PACKAGE"
colcon build
print_blue "SOURCING PACKAGE"
source install/setup.bash

print_blue "PACKAGES BUILT:"
ros2 pkg list | grep ${PACKAGE_NAME}
