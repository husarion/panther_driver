#!/usr/bin/python3

import canopen
from dataclasses import dataclass
from time import time, sleep
from threading import Thread, Lock

import rospy

from constants import LEFT_WHEEL, RIGHT_WHEEL, VOLT_CHANNEL, AMP_CHANNEL

@dataclass
class MotorController:
    def __init__(self, can_node_id, eds_file) -> None:
        self.wheel_pos = [0.0, 0.0]
        self.wheel_vel = [0.0, 0.0]
        self.wheel_curr = [0.0, 0.0]
        self.battery_data = [0.0, 0.0] # V, I
        self.runtime_stat_flag = [0, 0]
        self.fault_flags = 0

        self.can_node = canopen.RemoteNode(can_node_id, eds_file)


class PantherCAN:
    def __init__(self, eds_file, can_interface) -> None:
        self.can_net_err = False
        self._max_err_per_sec = 2
        self._err_times = [0] * self._max_err_per_sec
        
        self._lock = Lock()
        self._network = canopen.Network()

        self._motor_controllers = [
            MotorController(1, eds_file),   # front
            MotorController(2, eds_file)    # rear
        ]
        
        self._network.connect(channel=can_interface, bustype='socketcan')

        self._network.add_node(self._motor_controllers[0].can_node)
        self._network.add_node(self._motor_controllers[1].can_node)
        
        self._connection_check_timer = Thread(target=self._can_net_check)  
        self._connection_check_timer.start()

        rospy.loginfo(f'[{rospy.get_name()}] Connected to the CAN bus.')
        
    def set_wheels_enc_velocity(self, vel: list) -> None:
        with self._lock:
            for motor_controller, enc_vel in zip(self._motor_controllers, [vel[:2], vel[2:]]):
                for i, wheel in enumerate([LEFT_WHEEL, RIGHT_WHEEL]):
                    try:
                        motor_controller.can_node.sdo['Cmd_CANGO'][wheel].raw = enc_vel[i]
                    except:
                        rospy.logwarn(
                            f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError '
                            f'occurred while setting wheels velocity'
                        )
                        self._error_handle()

    def get_wheels_enc_pose(self):
        with self._lock:
            for motor_controller in self._motor_controllers:
                for i, wheel in enumerate([LEFT_WHEEL, RIGHT_WHEEL]):
                    try:
                        motor_controller.wheel_pos[i] = motor_controller.can_node.sdo['Qry_ABCNTR'][wheel].raw
                    except canopen.SdoCommunicationError:
                        rospy.logwarn(
                            f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError ' 
                            'occurred while reading wheels position'
                        )
                        self._error_handle()
                    yield motor_controller.wheel_pos[i]

    def get_wheels_enc_velocity(self):
        with self._lock:
            for motor_controller in self._motor_controllers:
                for i, wheel in enumerate([LEFT_WHEEL, RIGHT_WHEEL]):
                    try:
                        motor_controller.wheel_vel[i] = motor_controller.can_node.sdo['Qry_ABSPEED'][wheel].raw
                    except canopen.SdoCommunicationError:
                        rospy.logwarn(
                            f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError '
                            'occurred while reading wheels velocity'
                        )
                        self._error_handle()
                    yield motor_controller.wheel_vel[i]

    def get_motor_enc_current(self):
        with self._lock:
            for motor_controller in self._motor_controllers:
                for i, wheel in enumerate([LEFT_WHEEL, RIGHT_WHEEL]):
                    try:
                        # division by 10 is needed according to documentation
                        motor_controller.wheel_curr[i] = motor_controller.can_node.sdo['Qry_MOTAMPS'][wheel].raw / 10.0
                    except canopen.SdoCommunicationError:
                        rospy.logwarn(
                            f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError ' 
                            'occurred while reading motor current'
                        )
                        self._error_handle()
                    yield motor_controller.wheel_curr[i]

    def get_battery_data(self): 
        with self._lock:
            for motor_controller in self._motor_controllers:
                try:
                    # division by 10 is needed according to documentation
                    motor_controller.battery_data[0] = float(motor_controller.can_node.sdo['Qry_VOLTS'][VOLT_CHANNEL].raw) / 10.0
                    motor_controller.battery_data[1] = float(motor_controller.can_node.sdo['Qry_BATAMPS'][AMP_CHANNEL].raw) / 10.0
                except canopen.SdoCommunicationError:
                    rospy.logwarn(
                        f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError ' 
                        'occurred while reading battery data'
                    )       
                    self._error_handle()
                
                for i in range(2):
                    yield motor_controller.battery_data[i]

    def read_fault_flags(self):
        with self._lock:
            for motor_controller in self._motor_controllers:
                try:
                    motor_controller.fault_flags = motor_controller.can_node.sdo['Qry_FLTFLAG'].raw
                except canopen.SdoCommunicationError:
                    rospy.logwarn(
                        f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError occurred while reading fault flags'
                    )
                    self._error_handle()
                yield motor_controller.fault_flags

    def read_runtime_stat_flag(self):
        with self._lock:
            for motor_controller in self._motor_controllers:
                for i, wheel in enumerate([LEFT_WHEEL, RIGHT_WHEEL]):
                    try:
                        motor_controller.runtime_stat_flag[i] = int.from_bytes(motor_controller.can_node.sdo['Qry_MOTFLAGS'][wheel].data, 'little') 
                    except canopen.SdoCommunicationError:
                        rospy.logwarn(
                            f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError occurred while reading runtime status flag'
                        )
                        self._error_handle()
                    yield motor_controller.runtime_stat_flag[i]
        
    def _error_handle(self) -> None:
        self._err_times.append(time())
        self._err_times.pop(0)
    
    def _can_net_check(self) -> None:
        while True:
            self.can_net_err = True if (
                self._err_times[-1] - self._err_times[0] <= 1.0 and time() - self._err_times[-1] <= 2.0
            ) else False

            sleep(1)

    def _turn_on_roboteq_emergency_stop(self) -> None:
        with self._lock:
            for motor_controller in self._motor_controllers:
                motor_controller.can_node.sdo['Cmd_ESTOP'].raw = 1

    def _turn_off_roboteq_emergency_stop(self) -> None:
        with self._lock:
            for motor_controller in self._motor_controllers:
                motor_controller.can_node.sdo['Cmd_MGO'].raw = 1

        