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

### Kinematics type

Panther can be configured with different wheels to match your needs, we provide 3 different kinematics types `classic`/`mecanum`/`mix` you can change type by selecting appropriate parameter in launch file - `wheel_type`. 

Example launch file: 

```xml
<launch>
    <arg name="use_imu" default="false"/>
    <arg name="use_lights" default="false"/>
    <arg name="kalman_params" default="false"/>

    <group if="$(arg kalman_params)">
        <node pkg="panther_driver" name="panther_driver" type="driver_node.py" output="screen" required="true">
            <param name="can_interface" type="string" value="panther_can"/>
            <param name="wheel_type" type="string" value="classic"/>
            <!-- "classic" / "mecanum" / "mix" -->
            <param name="odom_frame" type="string" value="odom_wheel"/>
            <param name="base_link_frame" type="string" value="base_link"/>
            <param name="publish_tf" type="string" value="false"/>
            <param name="publish_pose" type="string" value="true"/>
            <param name="publish_odometry" type="string" value="true"/>

            <param name="robot_width" type="double" value="0.682"/>
            <param name="robot_length" type="double" value="0.44"/>
            <param name="wheel_radius" type="double" value="0.1825"/>
        
            <param name="eds_file" type="string" value="$(find panther_driver)/params/roboteq_motor_controllers_v60.eds"/>
        </node>
    </group>

    <group unless="$(arg kalman_params)">
        <node pkg="panther_driver" name="panther_driver" type="driver_node.py" output="screen" required="true">
            <param name="can_interface" type="string" value="panther_can"/>
            <param name="wheel_type" type="string" value="classic"/>
            <!-- "classic" / "mecanum" / "mix" -->
            <param name="odom_frame" type="string" value="odom"/>
            <param name="base_link_frame" type="string" value="base_link"/>
            <param name="publish_tf" type="string" value="true"/>
            <param name="publish_pose" type="string" value="true"/>
            <param name="publish_odometry" type="string" value="false"/>

            <param name="robot_width" type="double" value="0.682"/>
            <param name="robot_length" type="double" value="0.44"/>
            <param name="wheel_radius" type="double" value="0.1825"/>
        
            <param name="eds_file" type="string" value="$(find panther_driver)/params/roboteq_motor_controllers_v60.eds"/>
        </node>
    </group>

    <group if="$(arg use_lights)">
        <node pkg="panther_lights" name="lights_node" type="lights_node"/>
        <node pkg="panther_lights" name="lights_controller_simple" type="lights_controller_simple"/>
    </group>

    <group if="$(arg use_imu)">
        <include file="$(find phidgets_spatial)/launch/spatial.launch"></include>
    </group>

</launch>
```

For kalman filter setup please refer to [panther_ekf](https://github.com/adamkrawczyk/panther_ekf)

## Setup RAP startup in NUC

### rap script
cp ../system_config/rap.sh  /usr/bin/rap.sh
chmod a+x /usr/bin/rap.sh
### rap service
cp ../system_config/rap.service /lib/systemd/system/rap.service
systemctl enable rap.service


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
