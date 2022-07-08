#!/bin/bash

sudo cp power_gpio.py /usr/bin/power_gpio.py
sudo chmod +x power_gpio.py
sudo rm /etc/systemd/system/soft_stop_service.service
sudo cp soft_stop.service /etc/systemd/system/soft_stop.service 
sudo systemctl enable soft_stop.service
