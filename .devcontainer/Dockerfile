FROM osrf/ros:foxy-desktop

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Update all packages
RUN sudo apt update && sudo apt upgrade -y
# Set Git to automatically convert line endings to LF on checkout, and to CRLF on commit
RUN git config --global core.autocrlf true

# Install dos2unix, a tool to convert text files with DOS or MAC line breaks to Unix line breaks
RUN sudo apt-get install -y dos2unix

# Install usbutils, a set of utility programs for managing and inspecting USB devices
RUN sudo apt install -y usbutils

# Install linux-tools-virtual and hwdata, tools and data for working with hardware
RUN sudo apt install -y linux-tools-virtual hwdata

# Install net-tools, providing the ifconfig, netstat, route, arp, and other classic network admin tools
RUN sudo apt install -y net-tools

# Install iputils-ping, a tool for network diagnostics
RUN sudo apt install -y iputils-ping

# Install x11-apps, a set of client applications for X11 (the X Window System)
RUN sudo apt install -y x11-apps

# Install ROS packages for Gazebo, a 3D dynamic simulator with the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments
RUN sudo apt install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs

# Install xacro (XML Macro Language), a ROS package for generating XML files for ROS
RUN sudo apt install -y ros-${ROS_DISTRO}-xacro

# Install tf-transformations, a ROS package for handling 3D transform datatypes
RUN sudo apt install -y ros-${ROS_DISTRO}-tf-transformations

# Install joint-state-publisher, a ROS package for publishing `sensor_msgs/JointState` messages for a robot
RUN sudo apt install -y ros-${ROS_DISTRO}-joint-state-publisher

# Install slam-toolbox, a ROS package for 2D simultaneous localization and mapping
RUN sudo apt install -y ros-${ROS_DISTRO}-slam-toolbox

# Install nav2-bringup, a ROS package for bringing up all the navigation2 servers, plugins and configurations
RUN sudo apt install -y ros-${ROS_DISTRO}-nav2-bringup

# Install python3-pip, a package installer for Python
RUN sudo apt install -y python3-pip

# Install Python packages: numpy (for numerical operations), pandas (for data manipulation), opencv-python (for image and video processing), transforms3d (for 3D transformations), geopy (for geolocation)
RUN pip install numpy pandas opencv-python transforms3d geopy