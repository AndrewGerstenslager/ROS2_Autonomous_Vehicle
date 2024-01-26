source /opt/ros/${ROS_DISTRO}/setup.bash

colcon build
. install/setup.bash

ros2 pkg list | grep my_first_package
echo "EXECUTABLES:"
ros2 pkg executables my_first_package                 
echo "LAUNCH FILES:"
ls launch