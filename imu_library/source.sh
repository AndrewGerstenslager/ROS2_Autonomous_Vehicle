#THIS IS A CUSTOM INITALIZATION FILE
source /opt/ros/foxy/setup.bash

echo "BUILDING PACKAGE"
colcon build
echo "SOURCING PACKAGE"
. install/local_setup.bash

echo "PACKAGES BUILT:"
ros2 pkg list | grep sparkfun_imu_node
echo "DONE"

echo "EXECUTABLES:"
ros2 pkg executables sparkfun_imu_node    