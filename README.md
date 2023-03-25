# __DOKALMAN REPOSITORY__

### This is the repository for all current dokalman development

#### This software package is broken up into the different libraries for different parts of Dokalman's software environment

## __motor_library__

See specific documentation [here](/motor_library/README.md)

This library is designed to communicate from the Intel NUC to the onboard Arduino. The NUC uses serial communication over usb (UART) to send commands to the Arduino.

Arduino code and ros2 code is all located inside of this library

## __led_library - to be deprecated soon__

This code will be merged into the motor library to be run on the arduino

## __vision_library__

See specific documentation [here](/vision_library/README.md)

This library controls the whole vision pipeline from reading camera data, processing it, and converting that into a format usable by the navigation algorithms on the robot (see the library documentation for more details)

## __velodyne_library__

TODO: FIX LINK PATH BELOW
See specific documentation [here](/)

This library contains script files that allow dokalman to run the velodyne lidar using the velodyne ros2 package. It also contains the rviz2 script to launch rviz and in this library is a rviz2 config file that can be laoded in for quick setup

## __gps_library__

TODO: FIX LINK PATH BELOW
See specific documentation [here](/)

This library uses serial communication to connect to the onboard Novatel GPS and requests the unit to start logging the position (TODO: and potentially heading) data to the USB port. We read in that data and then we publish the data to ros2.  

## __roslaunch_files__

Here we are trying to collect the roslaunch files in one place. These are all bash (.sh) scripts or launch files (.xml). 

***NOTE: If you make new libraries, make sure to add a source.sh file to build and source the library. See the motor control file [here](/motor_library/source.sh) and add it [here](/roslaunch_files/build_and_source_all.sh) to keep everything updated!!***

