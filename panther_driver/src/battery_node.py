#!/usr/bin/python3

import math
import yaml

import rospy

from sensor_msgs.msg import BatteryState
from panther_msgs.msg import DriverStateArr


class PantherBatteryNode:
    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=False)

        measurements_file = rospy.get_param('~measurements_file')
        loop_rate = rospy.get_param('~loop_rate', 20)

        with open(measurements_file, 'r') as stream:
            try:
                self._config_file = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        
        self._A = 298.15
        self._B = 3950.0
        self._U_supply = 3.28
        self._R1 = 10000.0
        self._R0 = 10000.0

        self._V_driv_front = float('nan')
        self._V_driv_rear = float('nan')
        self._I_driv_front = float('nan')
        self._I_driv_rear = float('nan')

        # -------------------------------
        #   Publishers & Subscribers
        # -------------------------------

        self._battery_driv_sub = rospy.Subscriber('panther_driver/state', DriverStateArr, self._battery_driv_callback)

        self._battery_publisher = rospy.Publisher('battery', BatteryState, queue_size=1)
        self._battery1_publisher = rospy.Publisher('battery1', BatteryState, queue_size=1)
        self._battery2_publisher = rospy.Publisher('battery2', BatteryState, queue_size=1)

        # -------------------------------
        #   Timers
        # -------------------------------

        rospy.Timer(rospy.Duration(1.0 / loop_rate), self._battery_timer_callback)

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _battery_driv_callback(self, msg) -> None:
        self._V_driv_front = msg.front.voltage
        self._V_driv_rear = msg.front.current
        self._I_driv_front = msg.rear.voltage
        self._I_driv_rear = msg.rear.current
        
    def _battery_timer_callback(self, *args) -> None:
        try:
            V_bat1 = self._get_ADC_measurement('BAT1_voltage', self._config_file)
            V_bat2 = self._get_ADC_measurement('BAT2_voltage', self._config_file)
            V_temp_bat1 = self._get_ADC_measurement('BAT1_temp', self._config_file)
            V_temp_bat2 = self._get_ADC_measurement('BAT2_temp', self._config_file)
            I_charge_bat1 = self._get_ADC_measurement('BAT1_charge_current', self._config_file)
            I_charge_bat2 = self._get_ADC_measurement('BAT2_charge_current', self._config_file)
            I_bat1 = self._get_ADC_measurement('BAT1_current', self._config_file)
            I_bat2 = self._get_ADC_measurement('BAT2_current', self._config_file)
        except:
            rospy.logerr(f'[{rospy.get_name()}] Battery ADC measurement error excep')
            return

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

            V_bat_avereage = (V_bat1 + V_bat2) / 2.0
            temp_average = (temp_bat1 + temp_bat2) / 2.0
            I_bat_average = (I_bat1 + I_bat2) / 2.0
            I_charge_bat_average = (I_charge_bat1 + I_charge_bat2) / 2.0

            self._publish_battery_msg(
                self._battery_publisher, True, V_bat_avereage, temp_average, -I_bat_average + I_charge_bat_average
            )

    def _voltage_to_deg(self, V_temp) -> float:
        if V_temp == 0 or V_temp >= self._U_supply:
            rospy.logerr(f'[{rospy.get_name()}] Temperature measurement error')
            return float('nan')

        R_therm = (V_temp * self._R1) / (self._U_supply - V_temp)

        return (self._A * self._B / (self._A * math.log(R_therm / self._R0) + self._B)) - 273.15
        
    def _get_ADC_measurement(self, name: str, config_file) -> float:
        data = config_file[name]
        path = data['path']
        raw_value = self._read_file(path)
        value = (raw_value - data['offset']) * data['LSB']

        return value

    @staticmethod
    def _read_file(path) -> int:
        with open(path, 'r') as file:
            data = file.read().rstrip()
        file.close()

        return int(data)
    
    @staticmethod
    def _publish_battery_msg(bat_pub, present, V_bat=float('nan'), temp_bat=float('nan'), I_bat=float('nan')) -> None:
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
            battery_msg.voltage = float('nan')
            battery_msg.temperature = float('nan')
            battery_msg.current = float('nan')
            battery_msg.percentage = float('nan')
            battery_msg.capacity = float('nan')
            battery_msg.design_capacity = float('nan')
            battery_msg.charge = float('nan')
            battery_msg.power_supply_status
            battery_msg.power_supply_health
            battery_msg.power_supply_technology = 3
            battery_msg.present = False

        bat_pub.publish(battery_msg)


def main():
    panther_battery_node = PantherBatteryNode('panther_battery_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
