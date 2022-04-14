#!/bin/bash

sudo cp panther_can_setup.sh /usr/bin/panther_can_setup.sh
sudo chmod +x /usr/bin/panther_can_setup.sh
sudo cp panther_can_setup.service /etc/systemd/system/panther_can_setup.service
sudo systemctl enable panther_can_setup.service
