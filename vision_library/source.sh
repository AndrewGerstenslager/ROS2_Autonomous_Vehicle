source /opt/ros/foxy/setup.bash

colcon build
. install/setup.bash

ros2 pkg list | grep my_first_package
echo "EXECUTABLES:"
ros2 pkg executables my_first_package                 
