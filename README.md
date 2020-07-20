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

Use `update_startup.sh` to set required services:

```
cd scripts
sudo ./update_startup.sh
```

For omni wheel use `update_startup_omni.sh` to set required services:

```
cd scripts
sudo ./update_startup_omni.sh
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
