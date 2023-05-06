#THIS IS A CUSTOM INITALIZATION FILE
rm -r install


echo "BUILDING PACKAGE"
colcon build --packages-select dokalman
echo "SOURCING PACKAGE"
. install/local_setup.bash

echo "PACKAGES BUILT:"
ros2 pkg list | grep dokalman
echo "DONE"
echo "EXECUTABLES:"
ros2 pkg executables dokalman 
ros2 launch dokalman display.launch.py