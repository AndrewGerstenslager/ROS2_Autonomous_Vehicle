colcon build
. install/setup.bash

echo "PACKAGE BUILT"
ros2 pkg list | grep cpp_image_processing

echo "EXECUTABLES IN PACKAGE"
ros2 pkg executables cpp_image_processing

source install/setup.bash
