FROM ros:noetic-ros-core

ENV ROS_WS ros_ws
ARG run_tests=false

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

# Test dependencies
RUN if [[ "$run_tests" == 'true' ]] ; then \
        apt install -y cmake make g++ lcov gettext-base jq && \
        git clone -q https://github.com/google/googletest.git /googletest && \
        mkdir -p /googletest/build && \
        cd /googletest/build && \
        cmake -DBUILD_SHARED_LIBS=ON -Dgtest_build_samples=ON -G"Unix Makefiles" .. && \
        make && \
        make install && \
        cd / && \
        rm -rf /googletest ; \ 
    fi

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
RUN git clone https://github.com/husarion/husarion_msgs.git --branch dev src/husarion_msgs && \
    mkdir build && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep init && \
    rosdep update && \
    if [[ "$run_tests" == 'true' ]] ; \
    then \
        catkin_make -DCATKIN_ENABLE_TESTING=1 -DCMAKE_BUILD_TYPE=Debug ; \
    else \
        catkin_make -DCATKIN_ENABLE_TESTING=0 ; \
    fi

WORKDIR /

# Clear 
RUN apt clean && \
    rm -rf /var/lib/apt/lists/* 

COPY ./ros_entrypoint.sh /
