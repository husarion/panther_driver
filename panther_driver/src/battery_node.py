#!/usr/bin/python3

import math
import rospy
import canopen
from sensor_msgs.msg import BatteryState

import yaml

def read_file(path):
    with open(path, 'r') as file:
        data = file.read().rstrip()

    file.close()

    return int(data)

def get_measurement(name: str, config_file):
    data = config_file[name]
    path = data["path"]
    raw_value = read_file(path)
    value = raw_value * data["LSB"]

    return value



def driverNode():
    rospy.init_node('~', anonymous=False)

    battery_publisher = rospy.Publisher('battery', BatteryState, queue_size=1)
    battery1_publisher = rospy.Publisher('battery1', BatteryState, queue_size=1)
    battery2_publisher = rospy.Publisher('battery2', BatteryState, queue_size=1)

    can_interface = 'panther_can'


    if rospy.has_param('~eds_file'):
        eds_file = rospy.get_param('~eds_file')
    else:
        rospy.logerr("eds_file not defined, can not start CAN interface")
        return

    if rospy.has_param('~measurements_file'):
        measurements_file = rospy.get_param('~measurements_file')
    else:
        rospy.logerr("measurements_file not defined, can not start collecting ADC measurements")
        return


    loop_rate = 10
    rate = rospy.Rate(loop_rate)
    rospy.loginfo("Start with creating a network representing one CAN bus")
    network = canopen.Network()
    rospy.loginfo("Add some nodes with corresponding Object Dictionaries")
    # front_controller = canopen.RemoteNode(1, eds_file)
    # rear_controller = canopen.RemoteNode(2, eds_file)
    # network.add_node(front_controller)
    # network.add_node(rear_controller)
    # rospy.loginfo("Connect to the CAN bus")
    network.connect(channel=can_interface, bustype='socketcan')

    with open(measurements_file, "r") as stream:
        try:
            config_file = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    while not rospy.is_shutdown():
        try:
            rospy.get_master().getPid()
        except:
            rospy.logerr("Error getting master")
            exit(1)
        # try:
        #     try:
        #         battery_msg = BatteryState()
        #         battery_msg.voltage = float(
        #             front_controller.sdo[0x210D][2].raw)/10
        #         battery_msg.current = float(
        #             front_controller.sdo['Qry_BATAMPS'][1].raw)/10
        #         battery_publisher.publish(battery_msg)
        #     except:
        #         rospy.logwarn("Error getting battery data") 

        # except:
        #     rospy.logerr("CAN protocol error")

        try:
            battery1_msg = BatteryState()
            battery1_msg.header.stamp=rospy.Time.now()
            battery1_msg.voltage=get_measurement("BAT1_voltage", config_file)
            battery1_msg.temperature=get_measurement("BAT1_temp", config_file)
            battery1_msg.current=get_measurement("BAT1_charge_current", config_file)
            battery1_msg.percentage=(battery1_msg.voltage-32)/10
            battery1_msg.capacity=20
            battery1_msg.design_capacity=20
            battery1_msg.charge=battery1_msg.percentage*battery1_msg.design_capacity
            battery1_msg.power_supply_status
            battery1_msg.power_supply_health
            battery1_msg.power_supply_technology=3
            battery1_msg.present=True
            battery1_publisher.publish(battery1_msg)
        except:
            rospy.logerr("Battery 1 measurement error")

        try:
            battery2_msg = BatteryState()
            battery2_msg.header.stamp=rospy.Time.now()
            battery2_msg.voltage=get_measurement("BAT2_voltage", config_file)
            battery2_msg.temperature=get_measurement("BAT2_temp", config_file)
            battery2_msg.current=get_measurement("BAT2_charge_current", config_file)
            battery2_msg.percentage=(battery2_msg.voltage-32)/10
            battery2_msg.capacity=20
            battery2_msg.design_capacity=20
            battery2_msg.charge=battery2_msg.percentage*battery2_msg.design_capacity
            battery2_msg.power_supply_status
            battery2_msg.power_supply_health
            battery2_msg.power_supply_technology=3
            battery2_msg.present=True
            battery2_publisher.publish(battery2_msg)
        except:
            rospy.logerr("Battery 2 measurement error excep")

        # print("BAT1: ",get_measurement("BAT1_voltage", config_file), "BAT2: ",get_measurement("BAT2_voltage", config_file))

        rate.sleep()


if __name__ == '__main__':
    try:
        driverNode()
    except rospy.ROSInterruptException:
        pass
