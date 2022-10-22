# DOKALMAN REPOSITORY

### This is the repository for all current dokalman development

#### This software package is broken up into the different libraries for different parts of Dokalman

## Motor Library

See documentation [here](/motor_library/README.md)

This library is designed to communicate from the Intel NUC to the onboard Arduino

Arduino code and ros2 code is all located inside of this library

## Roslaunch Files

Here we are trying to collect the roslaunch files in one place. We are also trying to add systems in place to automatically build and source all of the libraries.

***NOTE: If you make new libraries, make sure to add a .sh file to build and source the library. See the motor control file [here](/motor_library/build_and_source.sh) and add it [here](/roslaunch_files/build_all.sh) to keep everything updated***

