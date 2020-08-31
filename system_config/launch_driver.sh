#!/bin/bash

eval "$(cat ~/.bashrc | tail -n +10)"
source /opt/ros/$(rosversion -d)/setup.sh
export ROS_IP=192.168.1.2
export ROS_MASTER_URI=http://192.168.1.3:11311
source /home/husarion/husarion_ws/devel/setup.sh
roslaunch panther_driver &

until ifconfig panther_can
do
  sleep 1
done
sleep 5
roslaunch panther_driver driver.launch
