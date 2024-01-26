#!/bin/bash

# Change the working directory to the location of the script
cd "$(dirname "${BASH_SOURCE[0]}")"

# THIS IS A CUSTOM INITIALIZATION FILE
#source /opt/ros/foxy/setup.bash

colcon build
source install/setup.bash

ros2 pkg list | grep dokalman_library
echo "EXECUTABLES:"
ros2 pkg executables dokalman_library              
echo "LAUNCH FILES:"
ls launch

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:worlds/gazebo_worlds/models