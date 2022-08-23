#!/usr/bin/python3

import canopen
from numpy import NaN

import rospy
from sensor_msgs.msg import BatteryState


class BatteryNode:
    def __init__(self, name):
        rospy.init_node(name)

        self._battery_publisher = rospy.Publisher('battery', BatteryState, queue_size=1)

        can_interface = rospy.get_param("~can_interface", "panther_can")

        if rospy.has_param("~eds_file"):
            eds_file = rospy.get_param("~eds_file")
        else:
            rospy.logerr(f"[{rospy.get_name()}] eds_file not defined, can not start CAN interface")
            return

        loop_rate = 20

        rospy.loginfo(
            f"[{rospy.get_name()}] Start with creating a network representing one CAN bus"
        )
        self.network = canopen.Network()
        rospy.loginfo(f"[{rospy.get_name()}] Add some nodes with corresponding Object Dictionaries")
        self.front_controller = canopen.RemoteNode(1, eds_file)
        self.rear_controller = canopen.RemoteNode(2, eds_file)
        self.network.add_node(self.front_controller)
        self.network.add_node(self.rear_controller)
        rospy.loginfo(f"[{rospy.get_name()}] Connect to the CAN bus")
        self.network.connect(channel=can_interface, bustype="socketcan")

        rospy.Timer(rospy.Duration(1 / loop_rate), self._battery_callback)

    def _battery_callback(self, *args):
        try:
            rospy.get_master().getPid()
        except:
            rospy.logerr(f"[{rospy.get_name()}] Error getting master")
            exit(1)

        try:
            # Publish battery data
            try:
                V_bat = float(self.front_controller.sdo[0x210D][2].raw) / 10
                I_bat = float(self.front_controller.sdo['Qry_BATAMPS'][1].raw) / 10
                self.publish_battery_msg(V_bat, I_bat)
            except:
                rospy.logwarn("Error getting battery data")
        except:
            rospy.logerr("CAN protocol error")

    def publish_battery_msg(self, V_bat=NaN, temp_bat=NaN, I_bat=NaN):
        battery_msg = BatteryState()

        battery_msg.header.stamp = rospy.Time.now()
        battery_msg.voltage = V_bat
        battery_msg.temperature = temp_bat
        battery_msg.current = I_bat
        battery_msg.percentage = (battery_msg.voltage - 32) / 10
        battery_msg.capacity = 20
        battery_msg.design_capacity = 20
        battery_msg.charge = battery_msg.percentage * battery_msg.design_capacity
        battery_msg.power_supply_status
        battery_msg.power_supply_health
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        battery_msg.present = True

        self._battery_publisher.publish(battery_msg)


def main():
    panther_battery = BatteryNode('panther_battery')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
