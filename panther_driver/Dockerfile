FROM ros:noetic-ros-core

ENV ROS_WS ros_ws

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update Ubuntu Software repository
RUN apt update && \
    apt upgrade -y

RUN apt install -y git \
    python3-dev \
    python3-pip \
    python3-rospkg \
    can-utils

# Python 3 dependencies
RUN pip3 install \
        rosdep \
        rospkg \
        canopen

# Create and initialise ROS workspace
RUN mkdir -p /$ROS_WS/src
COPY ./panther_driver /$ROS_WS/src/panther_driver
RUN chmod +x /$ROS_WS/src/panther_driver/src/driver_node.py
WORKDIR /$ROS_WS
RUN mkdir build && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep init && \
    rosdep update && \
    catkin_make -DCATKIN_ENABLE_TESTING=0

WORKDIR /

# Clear 
RUN apt clean && \
    rm -rf /var/lib/apt/lists/* 

COPY ./ros_entrypoint.sh /
