FROM ros:noetic-ros-core

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update Ubuntu Software repository
RUN apt update \
    && apt upgrade -y \
    && apt install -y git \
        openssh-server \
        python3-dev \
        python3-pip \
        python3-rospkg \
        python3-tf2-ros \
        ros-noetic-imu-filter-madgwick \
        ros-noetic-nodelet \
        ros-noetic-phidgets-drivers \
        ros-noetic-tf

# Python 3 dependencies
RUN pip3 install \
        rosdep \
        rospkg \
        canopen \
        RPi.GPIO \
        gpiozero \
        paramiko

WORKDIR /ros_ws

COPY ./panther_driver src/panther_driver

RUN git clone --branch ros1 https://github.com/husarion/panther_msgs.git src/panther_msgs

RUN mkdir build \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

# Clear 
RUN apt clean \
    && rm -rf /var/lib/apt/lists/* 

COPY ./ros_entrypoint.sh / 

ENTRYPOINT ["/ros_entrypoint.sh"]