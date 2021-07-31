# Velocity Manager

It's node dealing with many sources of /cmd_vel. Subscribes to /cmd_vel and publishes at */cmd_vel_filtered*. 

State machine got 4 states `AcceptAllState, DeadManState, JoyState, AutonomousState` in this order. 

To drive robot from joy hold `LB` (enable button) and left directional pad (D-pad). This will cause to going into JoyState.

Timeout is set to 5s and moves from JoyState an AutonomusState to AcceptAllState.

Going into DeadManState (To drive autonomously have to hold `LB + A`) is realized by pressing `X`. 

`Y` is a button which enables to get back to AcceptAllState (default state).

Schematic of state:

![drive-schematic](../docs/panther_controll.png)
### Drive speed

Control speed by pressing `RB` -> parking mode, and `RT` -> fast drive mode.

## ROS API

### Publish
  - `cmd_vel_filtered` *(geometry_msgs/Twist)* - filtered `cmd_vel` by topic.
  - `panther_driver/manager/status` *(husarion_msgs/PantherDriverStatus)* - status of state machine.

### Subscribe
  - `/cmd_vel` *(geometry_msgs/Twist)*


## Docker examples
### Panther hardware setup
``` bash
cd examples/panther_robot
docker-compose up --build
```

### Test software
``` bash
docker build -t panther-driver:test . --build-arg run_tests=true
```