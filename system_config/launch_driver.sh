#!/bin/bash

source /home/husarion/.bashrc
source /opt/ros/$(rosversion -d)/setup.sh
source /home/husarion/husarion_ws/devel/setup.sh
roslaunch panther_driver webui.launch &

until ifconfig panther_can
do
  sleep 1
done
sleep 5
roslaunch panther_driver driver.launch
