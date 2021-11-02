#!/bin/bash
# driver launch script
cp ../system_config/launch_driver.sh /usr/bin/launch_driver.sh
chmod a+x /usr/bin/launch_driver.sh
# driver service
cp ../system_config/launch_driver.service /lib/systemd/system/launch_driver.service
systemctl enable launch_driver.service
# can setup script
cp ../system_config/can_setup.sh  /usr/bin/can_setup.sh
chmod a+x /usr/bin/can_setup.sh
# can service
cp ../system_config/can-setup.service /lib/systemd/system/can-setup.service
systemctl enable can-setup.service
