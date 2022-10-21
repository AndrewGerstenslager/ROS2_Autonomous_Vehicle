echo "BUILDING PACKAGE"
colcon build
echo "SOURCING PACKAGE"
. install/local_setup.bash

echo "PACKAGES BUILT:"
ros2 pkg list | grep motor_control
echo "DONE"