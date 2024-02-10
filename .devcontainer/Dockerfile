# Base image
FROM osrf/ros:foxy-desktop

# Add vscode user with same UID and GID as your host system
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

# Install Git, a version control system
RUN sudo apt install -y git

# Update rosdep, a tool for installing system dependencies for ROS packages
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Andrew's additions
RUN git config --global core.autocrlf true \
    # dos2unix is used to convert text files from Windows format to Unix format
    && sudo apt-get install -y dos2unix \
    # usbutils provides lsusb utility for listing USB devices
    && sudo apt-get install -y usbutils \
    # linux-tools-virtual provides perf, a performance analyzing tool in Linux
    && sudo apt-get install -y linux-tools-virtual \
    # hwdata provides hardware identification and configuration data
    && sudo apt-get install -y hwdata \
    # net-tools provides the ifconfig utility for network interface configuration
    && sudo apt-get install -y net-tools \
    # iputils-ping provides the ping command
    && sudo apt-get install -y iputils-ping \
    # x11-apps provides a set of small applications which come with the X Window System
    && sudo apt-get install -y x11-apps \
    # ROS packages for Gazebo simulator, Xacro, TF transformations, Joint State Publisher, and SLAM toolbox
    && sudo apt install -y \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-slam-toolbox \
    # python3-pip is the package installer for Python
    && sudo apt install -y python3-pip

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Pip install all necessary packages
# numpy is a package for scientific computing with Python
# pandas is a data analysis and manipulation library
# opencv-python is a library for computer vision tasks
# transforms3d is a library for 3D transformations
# geopy is a client for several popular geocoding web services
RUN pip install \
    numpy \
    pandas \
    opencv-python \
    transforms3d \
    geopy