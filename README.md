# Panther driver

Software for controlling Panther robot motors via CAN interface.

## Installation

### Install Python canopen library:
It is a Pytrhon library for CAN inetrfaace

```
sudo pip install canopen
```

### Install 'can-utils' - driver for USB-CAN converter
```
cd ~
git clone https://github.com/vitroTV/can-utils.git
cd can-utils
make
```

### Install Husarion repositories
We are using web ui with simple joystic to send velocity commands and custom driver to translate ROS messages to CAN frames.
```
cd ~/husarion_ws/src
git clone https://github.com/husarion/webui-ros-joystick.git
git clone https://github.com/hsrn24/panther_driver.git
cd ~/husarion_ws
catkin_make
cd ~/husarion_ws/src/webui-ros-joystick/nodejs
npm install rosnodejs express socket.io yargs
```

## Set system services
This step is not mandatory, but will make easier robot up startup process.

### CAN port initialization

Edit `/usr/bin/can_setup.sh`:
```
sudo nano /usr/bin/can_setup.sh
```

Paste following content:
```
#!/bin/bash
modprobe can
modprobe can-raw
modprobe slcan
cd /home/husarion/can-utils
echo "slcan_attach..."
./slcan_attach -o -s4 -n panther_can /dev/ttyACM0
sleep 5
echo "slcand..."
./slcand -o -f -s4 -F /dev/ttyACM0 panther_can &
sleep 5
echo "ifconfig..."
ifconfig panther_can up
```

Make the file executable:
```
sudo chmod a+x /usr/bin/can_setup.sh
```

Edit `/lib/systemd/system/can-setup.service`:
```
sudo nano /lib/systemd/system/can-setup.service
```

Paste following content:
```
[Unit]
Description=Enable CAN port for Panther robot
After=syslog.target network.target multi-user.target nodm.service user@1000.service

[Service]
Type=simple
ExecStartPre=/bin/sleep 15
ExecStart=/usr/bin/can_setup.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Enable service:
```
sudo systemctl enable can-setup.service
```


# Roslaunch driver

Edit `/usr/bin/launch_driver.sh`:
```
sudo nano /usr/bin/launch_driver.sh
```

Paste following content:
```
#!/bin/bash
# delay start of driver
source /opt/ros/melodic/setup.sh
source /home/husarion/husarion_ws/devel/setup.sh
roslaunch panther_driver driver.launch
```

Make the file executable:
```
sudo chmod a+x /usr/bin/launch_driver.sh
```

Edit `/lib/systemd/system/launch_driver.service`:
```
sudo nano /lib/systemd/system/launch_driver.service
```

Paste following content:
```
[Unit]
Description=Launch Panther driver

[Service]
Type=simple
ExecStartPre=/bin/sleep 25
ExecStart=/usr/bin/launch_driver.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Enable service:
```
sudo systemctl enable launch_driver.service
```

## Usage
With both service added, driver and webui start with system boot.
Open PANTHER_IP:8000 and you will be able to drive robot with use of joystick.


## Viewing measurement data

User can preview some of the sensor data using command line
Connect with robot through SSH:

```
ssh husarion@PANTHER_IP
```

### Battery voltage
```
rostopic echo /battery
```

### Encoders, motor speed, motor current

```
rostopic echo /joint_states
```
You will see data structured as ROS message `sensor_msgs/JointState` with fields:

`position = [Front left, Front right, Rear left, Rear right]` - Encoder pulses

`velocity = [Front left, Front right, Rear left, Rear right]` - Encoder pulses per second

`effort = [Front left, Front right, Rear left, Rear right]` - Motor current in Amps

# Used docs
Documentation for USB-CAN converter:
https://ucandevices.github.io/uccb.html#!#socketCAN
