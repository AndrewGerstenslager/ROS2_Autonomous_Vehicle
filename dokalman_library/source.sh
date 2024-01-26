#!/bin/bash

#CHANGE LINE BELOW FOR UPDATING THE PACKAGE NAME
export PACKAGE_NAME='dokalman_library'

cd "$(dirname "${BASH_SOURCE[0]}")"

source /opt/ros/${ROS_DISTRO}/setup.bash

echo "BUILDING PACKAGE"
colcon build
echo "SOURCING PACKAGE"
source install/setup.bash

echo "PACKAGES BUILT:"
ros2 pkg list | grep ${PACKAGE_NAME}

# Check for executables
echo "EXECUTABLES:"
if ! ros2 pkg executables ${PACKAGE_NAME} | grep -q '.'; then
    echo "None"
fi

# Check for launch files
echo "LAUNCH FILES:"
if [ -z "$(ls -A launch 2>/dev/null)" ]; then
    echo "None"
else
    ls launch
fi

# Append to GAZEBO_MODEL_PATH. Adjust the path as necessary.
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/worlds/gazebo_worlds/models
