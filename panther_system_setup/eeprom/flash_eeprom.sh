#!/bin/sh

# Prepare i2c
dtparam=i2c_vc=on
sudo dtoverlay i2c-gpio i2c_gpio_sda=0 i2c_gpio_scl=1 bus=9

# Stop execution when command fails
set -e

# Save config dir
cd scripts/
script_dir=$PWD

cd ./../../config
config_dir=$PWD

# Generate /tmp/panther.eep file containing yaml file and hat serring coresponding panther version
sudo python3 $script_dir/generate_eeprom_file.py $config_dir

# Generate tmp blank file
cd /tmp && dd if=/dev/zero ibs=1k count=4 of=blank.eep

# Reset eeprom memory with blank file
cd /usr/local/sbin
sudo ./eepflash.sh -w -f=/tmp/blank.eep -t=24c32 -d=9 -a=50 -y

# Flash eeprom
sudo ./eepflash.sh -w -f=/tmp/panther.eep -t=24c32 -d=9 -a=50 -y

# Remove tmp files
sudo rm /tmp/panther.eep -f
sudo rm /tmp/blank.eep -f