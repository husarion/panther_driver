#!/bin/sh

SCRIPT_DIR=`dirname "$0"`
cd $SCRIPT_DIR/scripts/

# Install dependencies
./install_eepromutils.sh

# Copy files
sudo cp config_loading.py /usr/local/sbin/config_loading.py
sudo cp load_env_on_boot.sh /usr/local/sbin/load_env_on_boot.sh
sudo cp panther_eeprom_config.service /etc/systemd/system/panther_eeprom_config.service

# Set permissions
sudo chmod +x /usr/local/sbin/load_env_on_boot.sh

# Enable upstart
sudo systemctl enable panther_eeprom_config.service

echo "Will be loadig panther config on boot"
