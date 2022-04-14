#!/bin/sh

# Install dtc compiler and compile hat scripts
sudo apt install -y device-tree-compiler

cd ~/
sudo git clone https://github.com/raspberrypi/hats.git

# Install eeprom utils  
cd ~/hats/eepromutils
sudo make && sudo make install

cd ~/
sudo rm -rf hats

# Install dtoverlay
sudo chattr -a -i /etc/resolv.conf
sudo dpkg --configure resolvconf
sudo apt install -y libraspberrypi-bin
