#!/usr/bin/python3

import math
import yaml
from numpy import NaN

import rospy
from sensor_msgs.msg import BatteryState
from panther_msgs.msg import DriverStateArr


class BatteryNode(object):
    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=False)

        assert rospy.has_param(
            "~measurements_file"
        ), f"[{rospy.get_name()}] measurements_file not defined, can not start collecting ADC measurements"

        measurements_file = rospy.get_param('~measurements_file')
        loop_rate = rospy.get_param('~loop_rate', 20)

        with open(measurements_file, 'r') as stream:
            try:
                self.config_file = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        self.V_driv_front = NaN
        self.V_driv_rear = NaN
        self.I_driv_front = NaN
        self.I_driv_rear = NaN

        self._battery_driv_sub = rospy.Subscriber('panther_driver/state', DriverStateArr, self._battery_driv_callback)

        self._battery_publisher = rospy.Publisher('battery', BatteryState, queue_size=1)
        self._battery1_publisher = rospy.Publisher('battery1', BatteryState, queue_size=1)
        self._battery2_publisher = rospy.Publisher('battery2', BatteryState, queue_size=1)

        rospy.Timer(rospy.Duration(1 / loop_rate), self._battery_timer_callback)

    def _battery_driv_callback(self, msg) -> None:
        self.V_driv_front = msg.front.voltage
        self.V_driv_rear = msg.front.current
        self.I_driv_front = msg.rear.voltage
        self.I_driv_rear = msg.rear.current
        
    def _battery_timer_callback(self, *args) -> None:
        try:
            rospy.get_master().getPid()
        except:
            rospy.logerr(f'[{rospy.get_name()}] Error getting master')
            exit(1)

        try:
            V_bat1 = self._get_ADC_measurement('BAT1_voltage', self.config_file)
            V_bat2 = self._get_ADC_measurement('BAT2_voltage', self.config_file)
            V_temp_bat1 = self._get_ADC_measurement('BAT1_temp', self.config_file)
            V_temp_bat2 = self._get_ADC_measurement('BAT2_temp', self.config_file)
            I_charge_bat1 = self._get_ADC_measurement('BAT1_charge_current', self.config_file)
            I_charge_bat2 = self._get_ADC_measurement('BAT2_charge_current', self.config_file)
            I_bat1 = self._get_ADC_measurement('BAT1_current', self.config_file)
            I_bat2 = self._get_ADC_measurement('BAT2_current', self.config_file)
        except:
            rospy.logerr(f'[{rospy.get_name()}] Battery ADC measurement error excep')

        try:
            # Check battery num
            if V_temp_bat2 > 3.03:  # One battery
                # Calculate Temp in deg of Celcius
                temp_bat1 = self._voltage_to_deg(V_temp_bat1)

                self._publish_battery_msg(self._battery1_publisher, True, V_bat1, temp_bat1, I_bat1)
                self._publish_battery_msg(self._battery2_publisher, False)
            
            else:
                # Calculate Temp in deg of Celcius
                temp_bat1 = self._voltage_to_deg(V_temp_bat1)
                temp_bat2 = self._voltage_to_deg(V_temp_bat2)

                self._publish_battery_msg(
                    self._battery1_publisher, True, V_bat1, temp_bat1, -I_bat1 + I_charge_bat1
                )
                self._publish_battery_msg(
                    self._battery2_publisher, True, V_bat2, temp_bat2, -I_bat2 + I_charge_bat2
                )

                V_bat_avereage = (V_bat1 + V_bat2) / 2
                temp_average = (temp_bat1 + temp_bat2) / 2
                I_bat_average = (I_bat1 + I_bat2) / 2
                I_charge_bat_average = (I_charge_bat1 + I_charge_bat2) / 2

                self._publish_battery_msg(
                    self._battery_publisher, True, V_bat_avereage, temp_average, -I_bat_average + I_charge_bat_average
                )
        except:
            rospy.logerr(f'[{rospy.get_name()}] Error calculating and publishing bat data')

    def _get_ADC_measurement(self, name: str, config_file):
        data = config_file[name]
        path = data['path']
        raw_value = self._read_file(path)
        value = (raw_value - data['offset']) * data['LSB']

        return value

    def _read_file(self, path):
        with open(path, 'r') as file:
            data = file.read().rstrip()
        file.close()

        return int(data)

    def _voltage_to_deg(self, V_temp):
        # Source: 
        # https://electronics.stackexchange.com/questions/323043/how-to-calculate-temperature-through-ntc-thermistor-without-its-datasheet

        A = 298.15
        B = 3950
        U_supply = 3.28
        R1 = 10000
        R0 = 10000

        if V_temp == 0 or V_temp >= U_supply:
            rospy.logerr(f'[{rospy.get_name()}] Temperature measurement error')
            return NaN

        R_therm = (V_temp * R1) / (U_supply - V_temp)

        return (A * B / (A * math.log(R_therm / R0) + B)) - 273.15

    def _publish_battery_msg(self, bat_pub, present, V_bat=NaN, temp_bat=NaN, I_bat=NaN):
        battery_msg = BatteryState()
        if present:
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
            battery_msg.power_supply_technology = 3
            battery_msg.present = True
        else:
            battery_msg.header.stamp = rospy.Time.now()
            battery_msg.voltage = NaN
            battery_msg.temperature = NaN
            battery_msg.current = NaN
            battery_msg.percentage = NaN
            battery_msg.capacity = NaN
            battery_msg.design_capacity = NaN
            battery_msg.charge = NaN
            battery_msg.power_supply_status
            battery_msg.power_supply_health
            battery_msg.power_supply_technology = 3
            battery_msg.present = False

        bat_pub.publish(battery_msg)


def main():
    panther_battery_node = BatteryNode('panther_battery')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
