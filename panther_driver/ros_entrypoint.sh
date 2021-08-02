#!/bin/bash
set -e

# setup ros environment
source "/ros_ws/devel/setup.bash"
bash /ros_ws/src/panther_driver/scripts/update_startup.sh
exec "$@"