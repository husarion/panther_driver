#!/bin/bash

sudo cp panther_powerboard.py /usr/bin/panther_powerboard.py
sudo chmod +x /usr/bin/panther_powerboard.py
sudo rm /etc/systemd/system/soft_stop_service.service
sudo cp panther_powerboard.service /etc/systemd/system/panther_powerboard.service
sudo systemctl enable panther_powerboard.service
