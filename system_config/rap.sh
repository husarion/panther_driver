#!/bin/bash

source /home/husarion/.bashrc
source /opt/ros/$(rosversion -d)/setup.sh
source /home/husarion/ros_workspace/devel/setup.sh
roslaunch panther_driver rap.launch
