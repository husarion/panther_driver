#!/bin/sh

# Run script to generate export file
sudo python3 /usr/local/sbin/config_loading.py

# Set exec permission to export fille
sudo chmod +x /etc/profile.d/panther_env_export.sh 
