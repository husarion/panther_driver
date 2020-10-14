#!/bin/bash

eval "$(cat ~/.bashrc | tail -n +10)"
source /opt/ros/noetic/setup.sh
export ROS_IP=10.15.20.2
export ROS_MASTER_URI=http://10.15.20.3:11311
source /home/husarion/husarion_ws/devel/setup.sh

until ntpdate -u 10.15.20.3
do 
  sleep 1 
done 

until ifconfig panther_can && rostopic list
do
  sleep 1
done
sleep 5
roslaunch panther_driver driver.launch

