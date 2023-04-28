#THIS IS A CUSTOM INITALIZATION FILE

echo "BUILDING PACKAGE"
colcon build
echo "SOURCING PACKAGE"
. install/local_setup.bash

echo "PACKAGES BUILT:"
ros2 pkg list | grep gps_package
echo "EXECUTABLES"
ros2 pkg executables gps_package
echo "DONE"