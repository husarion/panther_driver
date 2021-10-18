FROM ros:noetic-ros-core

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update Ubuntu Software repository
RUN apt update \
    && apt upgrade -y

RUN apt install -y git \
    python3-dev \
    python3-pip \
    python3-rospkg \
    python3-tf2-ros \
    can-utils \
    net-tools

# Python 3 dependencies
RUN pip3 install \
        rosdep \
        rospkg \
        canopen
        

# Create and initialise ROS workspace
RUN mkdir -p /ros_ws/src
COPY ./panther_driver /ros_ws/src/panther_driver
RUN chmod +x /ros_ws/src/panther_driver/src/driver_node.py \
    && git clone https://github.com/adamkrawczyk/panther_ekf.git --single-branch --branch=master /ros_ws/src/panther_ekf

WORKDIR /ros_ws
RUN mkdir build \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep init \
    && rosdep update \
    && catkin_make -DCATKIN_ENABLE_TESTING=0

WORKDIR /

# Clear 
RUN apt clean \
    && rm -rf /var/lib/apt/lists/* 

COPY ./ros_entrypoint.sh /
