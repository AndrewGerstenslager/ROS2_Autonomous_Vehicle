#THIS FILE CAN BE USED TO BUILD EVERYTHING 
#FROM THE ROSLAUNCH FOLDER


#Build and source the motor library
cd ../motor_library/
. source.sh
cd ../roslaunch_files

# Build and source the vision library
cd ../vision_library
. source.sh
cd ../roslaunch_files

