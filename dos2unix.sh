# Run this if you are running source.sh files and are using docker containers and have issues executing them

# If this file does not run for the same reason just run the following command in the root folder of dokalman
find . -type f -name "*.sh" -exec dos2unix {} +

# Ignore any errors if there is something in the build, install, or ./roslaunch_files those shouldn't really matter for us