FROM ros:noetic-ros-core

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update Ubuntu Software repository
RUN apt update \
    && apt upgrade -y \
    && apt install -y git \
        python3-dev \
        python3-pip \
        python3-rospkg \
        python3-tf2-ros \
        ros-noetic-imu-filter-madgwick \
        ros-noetic-nodelet \
        ros-noetic-phidgets-drivers \
        ros-noetic-tf \
        ros-noetic-robot-localization

# Python 3 dependencies
RUN pip3 install \
        rosdep \
        rospkg \
        canopen
        
WORKDIR /app

# Create and initialise ROS workspace
RUN mkdir -p ros_ws/src
COPY ./panther_driver ros_ws/src/panther_driver
RUN chmod +x ros_ws/src/panther_driver/src/driver_node.py

RUN cd ros_ws \
    && mkdir build \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep init \
    && rosdep update \
    && catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release


# Clear 
RUN apt clean \
    && rm -rf /var/lib/apt/lists/* 

COPY ./ros_entrypoint.sh / 
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]