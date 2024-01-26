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

# Andrew added some stuff here
RUN git config --global core.autocrlf true
RUN sudo apt-get install -y dos2unix
RUN sudo apt install -y usbutils
RUN sudo apt install -y linux-tools-virtual hwdata
RUN sudo apt install -y net-tools
RUN sudo apt install -y iputils-ping

RUN sudo apt install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs
RUN sudo apt install -y ros-${ROS_DISTRO}-xacro
RUN sudo apt-get install -y python3-pip

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Pip install all necessary packages
RUN pip install numpy pandas opencv-python