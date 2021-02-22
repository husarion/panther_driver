#!/usr/bin/python3

import sys
import subprocess
import os

possible_arg_count = [2, 4]
if (len(sys.argv) not in possible_arg_count) or len(sys.argv) == 1:
    sys.exit("""
    ###Mismatch argument count. Expected 4.###

This script set up autostart of panther driver node for controlling lights, wheels, imu
USAGE: 
    sudo python3 set_driver_startup.py <username> <local_ip_addr> <ros_master_ip/ros_master_hostname>
    
Example for default panther configuration: 
    sudo python3 set_driver_startup.py husarion 10.15.20.2 10.15.20.3 

Uninstall: 
    sudo python3 set_driver_startup.py uninstall

WARNING: 
    In case of using before script "update_startup.sh" make sure to execute following commands:
    sudo systemctl disable launch_driver.service && sudo systemctl stop launch_driver.service
    sudo systemctl disable can-setup.service && sudo systemctl stop can-setup.service
""")


def prompt_sudo():
    ret = 0
    if os.geteuid() != 0:
        msg = "[sudo] password for %u:"
        ret = subprocess.check_call("sudo -v -p '%s'" % msg, shell=True)
    return ret


if prompt_sudo() != 0:
    sys.exit("The user wasn't authenticated as a sudoer, exiting")

if str(sys.argv[1]) == "uninstall":
    subprocess.call("rm /usr/sbin/can_setup.sh", shell=True)
    subprocess.call("rm /usr/sbin/driver_script.sh", shell=True)
    subprocess.call("rm /etc/ros/env.sh", shell=True)
    subprocess.call("rm /etc/systemd/system/can_setup.service", shell=True)
    subprocess.call(
        "rm /etc/systemd/system/panther_driver.service", shell=True)
    subprocess.call("systemctl disable can_setup.service", shell=True)
    subprocess.call("systemctl disable panther_driver.service", shell=True)
    sys.exit("All removed")

HOSTNAME = str(sys.argv[1])
ROS_IP = str(sys.argv[2])
ROS_MASTER_URI = str(sys.argv[3])


print("Configuration ->", "Hostname:", HOSTNAME, "ROS_IP:", ROS_IP,
      "ROS_MASTER_URI:", ROS_MASTER_URI)
subprocess.call("mkdir /etc/ros", shell=True)


#
# /etc/ros/env.sh
#

env_msg = """#!/bin/sh
export ROS_IP={rip} 
export ROS_MASTER_URI=http://{rmu}:11311
""".format(rip=ROS_IP, rmu=ROS_MASTER_URI)

subprocess.Popen(['echo "{}" > /etc/ros/env.sh'.format(env_msg)],  shell=True)


#
# /etc/systemd/system/can_setup.service
#

can_setup = """#!/bin/bash
modprobe can
modprobe can-raw
modprobe slcan
# cd /home/{hn}/can-utils
echo "slcan_attach..."
slcan_attach -o -s8 -n panther_can /dev/ttyACM0
sleep 5
echo "slcand..."
slcand -o -f -s8 -F /dev/ttyACM0 panther_can &
sleep 5
echo "ifconfig..."
ifconfig panther_can up""".format(hn=HOSTNAME)

can_service = """[Unit]
Description=Enable CAN port for Panther robot
After=syslog.target network.target multi-user.target nodm.service user@1000.service

[Service]
Type=simple
ExecStartPre=/bin/sleep 15
ExecStart=/usr/sbin/can_setup.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
"""

subprocess.Popen(
    ['echo "{}" > /usr/sbin/can_setup.sh'.format(can_setup)],  shell=True)
subprocess.call("touch /etc/systemd/system/can_setup.service", shell=True)
subprocess.Popen(
    ['echo "{}" > /etc/systemd/system/can_setup.service'.format(can_service)],  shell=True)


#
# /usr/sbin/panther_driver.sh
#

driver_script = """#!/bin/bash
. /opt/ros/noetic/setup.sh 
source /home/{hn}/husarion_ws/devel/setup.bash
source /etc/ros/env.sh
export ROS_HOME=$(echo ~{hn})/.ros

until ntpdate -u {rmu}
do 
  sleep 1 
done 

until ifconfig panther_can && rostopic list
do
  sleep 1
done
sleep 5
roslaunch panther_driver driver.launch
""".format(hn=HOSTNAME,rmu=ROS_MASTER_URI)

subprocess.Popen(
    ['echo "{}" > /usr/sbin/driver_script.sh'.format(driver_script)],  shell=True)


#
# driver_service 
#

driver_service = """
[Unit]
Description=Launch Panther driver
After=NetworkManager.service time-sync.target
[Service]
Type=simple
ExecStart=/usr/sbin/driver_script.sh
RemainAfterExit=yes
Restart=always

[Install]
WantedBy=multi-user.target
""".format(hn=HOSTNAME)

subprocess.Popen(
    ['echo "{}" > /etc/systemd/system/panther_driver.service'.format(driver_service)],  shell=True)



#
# soft stop
#

soft_stop_script = """#!/usr/bin/python3

import RPi.GPIO as GPIO
import time

input_pin = 15
output_pin = 31

GPIO.setmode(GPIO.BOARD)
GPIO.setup(output_pin, GPIO.OUT)
GPIO.setup(input_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

while(True):
    try:
        GPIO.output(output_pin, GPIO.input(input_pin))
        time.sleep(0.01)
    except:
        GPIO.cleanup()
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(output_pin, GPIO.OUT)
        GPIO.setup(input_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.cleanup()
"""

subprocess.Popen(
    ['echo "{}" > /usr/sbin/soft_stop_script.py'.format(soft_stop_script)],  shell=True)
#
# soft_stop_service 
#

soft_stop_service = """
[Unit]
Description=Set soft stop pin
After=NetworkManager.service time-sync.target
[Service]
Type=simple
ExecStart=/usr/bin/python3 /usr/sbin/soft_stop_script.py
RemainAfterExit=yes
Restart=always

[Install]
WantedBy=multi-user.target
"""

subprocess.Popen(
    ['echo "{}" > /etc/systemd/system/soft_stop_service.service'.format(soft_stop_service)],  shell=True)

subprocess.call("systemctl enable soft_stop_service.service", shell=True)
subprocess.call("chmod +x /usr/sbin/soft_stop_script.py", shell=True)
subprocess.call("systemctl enable can_setup.service", shell=True)
subprocess.call("chmod +x /usr/sbin/can_setup.sh", shell=True)
subprocess.call("systemctl enable panther_driver.service", shell=True)
subprocess.call("chmod +x /usr/sbin/driver_script.sh", shell=True)


print("Done!")
