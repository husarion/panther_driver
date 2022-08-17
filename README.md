# Panther driver

Software for controlling Panther robot motors via CAN interface.

## Installation

### Install Python canopen library:
It is a Python library for CAN inetrfaace

```
sudo pip install canopen
```

### Install 'can-utils' - driver for USB-CAN converter
```
sudo apt install can-utils
```

### Install Husarion repositories
We are using web ui with simple joystick to send velocity commands and custom driver to translate ROS messages to CAN frames.
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

### CAN bitrate
Slcan tool take `-sX` argument to set CAN bitrate. Below table contains valid values.

| ASCII Command | CAN Bitrate |
| ---           | ---         |
| s0            | 10 Kbit/s   |
| s1            | 20 Kbit/s   |
| s2            | 50 Kbit/s   |
| s3            | 100 Kbit/s  |
| s4            | 125 Kbit/s  |
| s5            | 250 Kbit/s  |
| s6            | 500 Kbit/s  |
| s7            | 800 Kbit/s  |
| s8            | 1000 Kbit/s |

## ROS API

### Published Topics

 * /battery [sensor_msgs/BatteryState]

 * /joint_states [sensor_msgs/JointState]
 
 * /pose [geometry_msgs/Pose]
 
 * /tf [tf2_msgs/TFMessage]
 
 * /odom/wheel [nav_msgs/Odometry] - default not active


### Subscribed Topics

* /cmd_vel [geometry_msgs/Twist]

### Parameters

`~wheel_type` (string, default: offroad)

    Specifies wheel type. Wheel types: 'offroad' 'small_pneumatic' 'mecanum'. Automatically changes kinematics type.

`~use_imu` (bool, default: true)

    Enable/Disable IMU

`~use_lights` (bool, default: true)

    Enable/Disable light panels

`~webui_joy` (bool, default: true)

    Enable/Disable webui joystick

`~kalman_params` (bool, default: true)

    Enable/Disable kalman filter

`~joy` (bool, default: true)

    Enable/Disable joystick
    
## Setup autostart

# Fix set_driver_startup.py

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
