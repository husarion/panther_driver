#!/bin/bash
modprobe can
modprobe can-raw
modprobe slcan
# cd /home/husarion/can-utils
echo "slcan_attach..."
slcan_attach -o -s8 -n panther_can /dev/ttyACM0
sleep 5
echo "slcand..."
slcand -o -f -s8 -F /dev/ttyACM0 panther_can &
sleep 5
echo "ifconfig..."
ifconfig panther_can up