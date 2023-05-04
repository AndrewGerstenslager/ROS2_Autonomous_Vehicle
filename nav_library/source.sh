#THIS IS A CUSTOM INITALIZATION FILE

echo "BUILDING PACKAGE"
colcon build --packages-select dokalman_model_package
echo "SOURCING PACKAGE"
. install/local_setup.bash

echo "PACKAGES BUILT:"
ros2 pkg list | grep dokalman_model_package
echo "DONE"
echo "EXECUTABLES:"
ros2 pkg executables dokalman_model_package    