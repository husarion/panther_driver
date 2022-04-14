sudo apt install -y \
                 python3-pip\
                 rpi-eeprom
sudo rpi-eeprom-config --apply boot.conf
sudo pip3 install RPi.GPIO
chmod +x power_gpio.py

# sudo groupadd gpio
# sudo adduser $USER gpio
# sudo chown root.gpio /dev/gpiomem
# sudo chmod -R g+rw /dev/gpiomem

sudo touch /etc/rc.local
sudo chmod +x /etc/rc.local

sudo sh -c "echo '#!/bin/sh -e' >> /etc/rc.local"
sudo sh -c "echo 'python3 ~/powerboard_rpi/power_gpio.py > ~/powerboard_rpi/startup.log 2>&1' >> /etc/rc.local"
