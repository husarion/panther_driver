FROM ros:noetic-ros-core

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update Ubuntu Software repository
RUN apt update && \
    apt upgrade -y

RUN apt install -y git \
    python3-dev \
    python3-pip \
    python3-rospkg \
    libgmock-dev

# Python 3 dependencies
RUN pip3 install \
        rosdep \
        rospkg


# Create and initialise ROS workspace
RUN mkdir -p /ros_ws/src
COPY ./velocity_manager /ros_ws/src/velocity_manager
WORKDIR /ros_ws
RUN git clone https://github.com/husarion/husarion_msgs.git --branch dev src/husarion_msgs
RUN mkdir build
RUN source /opt/ros/$ROS_DISTRO/setup.bash && rosdep init
RUN source /opt/ros/$ROS_DISTRO/setup.bash && rosdep update
# RUN source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make
RUN source /opt/ros/$ROS_DISTRO/setup.bash &&  catkin_make -DCATKIN_ENABLE_TESTING=1
RUN source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make run_tests

# run tests
RUN source /ros_ws/devel/setup.bash && \
    rosrun velocity_manager VelocityManagerTest

WORKDIR /

# Clear 
RUN apt clean && \
    rm -rf /var/lib/apt/lists/* 

COPY ./ros_entrypoint.sh /
