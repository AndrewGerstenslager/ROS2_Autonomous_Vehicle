#!/bin/bash

# CHANGE LINE BELOW FOR UPDATING THE PACKAGE NAME
export PACKAGE_NAME='motor_control'
# ADD LINE BELOW TO DEFINE THE PACKAGE TYPE
export PACKAGE_TYPE='python' # 'cpp' or 'python' depending on your package

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

# Check for executables
print_blue "EXECUTABLES:"
EXECUTABLES=$(ros2 pkg executables ${PACKAGE_NAME})
if [ -z "$EXECUTABLES" ]; then
    print_red "None"
else
    echo "$EXECUTABLES"
fi

# Check for launch files
print_blue "LAUNCH FILES:"
LAUNCH_DIR="src/${PACKAGE_NAME}/launch" # Default path for C++ packages
if [ "${PACKAGE_TYPE}" = "python" ]; then
    LAUNCH_DIR="launch" # Adjust path for Python packages if necessary
fi

if [ -z "$(ls -A ${LAUNCH_DIR} 2>/dev/null)" ]; then
    print_red "None"
else
    ls ${LAUNCH_DIR}
fi
