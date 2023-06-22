#!/bin/bash

# Convert xacro to urdf
ros2 run xacro xacro -o dokalman_description.urdf dokalman_description.xacro 

# Replace package:// URIs with absolute paths
#PACKAGE_PATH=$(ros2 pkg prefix dokalman | sed -n 's/^share: \(.*\)/\1/p')
#sed -i "s|package://dokalman|$PACKAGE_PATH|" dokalman_description.urdf

# Convert urdf to sdf
gz sdf -p dokalman_description.urdf > dokalman_description.sdf

#gazebo --verbose -s libgazebo_ros_factory.so path/to/your/output_sdf.sdf