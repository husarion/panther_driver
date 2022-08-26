#!/usr/bin/python3

from numpy import NaN

import rospy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32MultiArray


class BatteryNode:
    def __init__(self, name):
        rospy.init_node(name)

        self._battery_driv_sub = rospy.Subscriber(
            'battery_driv', Float32MultiArray, self._batter_driv_callback
        )

        self._battery_publisher = rospy.Publisher('battery', BatteryState, queue_size=1)

    def _batter_driv_callback(self, msg):
        V_bat = (msg.data[0] + msg.data[2]) / 2
        I_bat = (msg.data[1] + msg.data[3]) / 2
        self._publish_battery_msg(V_bat, I_bat)

    def _publish_battery_msg(self, V_bat=NaN, I_bat=NaN):
        battery_msg = BatteryState()

        battery_msg.header.stamp = rospy.Time.now()
        battery_msg.voltage = V_bat
        battery_msg.temperature = NaN
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
