#!/bin/bash
set -e

# setup ros environment
source "/ros_ws/devel/setup.bash"
# slcan_attach -o -s8 /dev/ttyACM0
# slcand -o -f -s8 -F /dev/ttyACM0
# ifconfig panther_can up
exec "$@"