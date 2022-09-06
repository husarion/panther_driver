#!/usr/bin/python3

import canopen
from time import sleep

import rospy

class CanErrorMsgs:
    wheels_names = [
        'front right wheel', 
        'front left wheel', 
        'rare right wheel', 
        'rare left wheel'
    ]

    fault_flags = [
        'f1: Overheat',
        'f2: Overvoltage',
        'f3: Undervoltage',
        'f4: Short circuit',
        'f5: Emergency stop',
        'f6: Motor/Sensor Setup fault',
        'f7: MOSFET failure',
        'f8: Default configuration loaded at startup'
    ]

    runtime_status_flags = [
        'rf1: Amps Limit currently active',
        'rf2: Motor stalled',
        'rf3: Loop Error detected',
        'rf4: Safety Stop active',
        'rf5: Forward Limit triggered',
        'rf6: Reverse Limit triggered',
        'rf7: Amps Trigger activated'
    ]


class PantherCAN:
    def __init__(self, eds_file, can_interface):
        rospy.loginfo(
            f"[{rospy.get_name()}] Start with creating a network representing one CAN bus."
        )

        self.error_cnt = 0
        self.error_max_cnt = 25
        self.lock = False

        self.network = canopen.Network()

        self.front_controller = canopen.RemoteNode(1, eds_file)
        self.rear_controller = canopen.RemoteNode(2, eds_file)

        self.network.add_node(self.front_controller)
        self.network.add_node(self.rear_controller)

        self.network.connect(channel=can_interface, bustype="socketcan")
        rospy.loginfo(f"[{rospy.get_name()}] Connected to the CAN bus.")

        # !TODO
        #   po wywołaniu estopu trzeba sprawdzić czy roboteqi są w estopie. Jeżeli nie są to trzeba coś zrobić

    def set_wheels_enc_velocity(self, fl_vel, fr_vel, rl_vel, rr_vel):
        try:
            self.front_controller.sdo["Cmd_CANGO"][2].raw = fl_vel
            self.front_controller.sdo["Cmd_CANGO"][1].raw = fr_vel
        except canopen.SdoCommunicationError:
            rospy.logwarn(f"[{rospy.get_name()}] PantherCAN: SdoCommunicationError occurred while writing enc speed (front controller)")
            self._error_handle()

        try:
            self.rear_controller.sdo["Cmd_CANGO"][2].raw = rl_vel
            self.rear_controller.sdo["Cmd_CANGO"][1].raw = rr_vel
        except canopen.SdoCommunicationError:
            rospy.logwarn(f"[{rospy.get_name()}] PantherCAN: SdoCommunicationError occurred while writing enc speed (rear controller)")
            self._error_handle()

    def get_wheels_enc_pose(self):
        wheel_pos = [0, 0, 0, 0]

        try:
            wheel_pos[0] = self.front_controller.sdo["Qry_ABCNTR"][2].raw
            wheel_pos[1] = self.front_controller.sdo["Qry_ABCNTR"][1].raw
        except canopen.SdoCommunicationError:
            rospy.logwarn(f"[{rospy.get_name()}] PantherCAN: SdoCommunicationError occurred while reading wheels position (front controller)")
            self._error_handle()

        try:
            wheel_pos[2] = self.rear_controller.sdo["Qry_ABCNTR"][2].raw
            wheel_pos[3] = self.rear_controller.sdo["Qry_ABCNTR"][1].raw
        except canopen.SdoCommunicationError:
            rospy.logwarn(f"[{rospy.get_name()}] PantherCAN: SdoCommunicationError occurred while reading wheels position (rear controller)")
            self._error_handle()

        return wheel_pos

    def get_wheels_enc_velocity(self):
        wheel_vel = [0, 0, 0, 0]

        try:
            wheel_vel[0] = self.front_controller.sdo["Qry_ABSPEED"][2].raw
            wheel_vel[1] = self.front_controller.sdo["Qry_ABSPEED"][1].raw
        except canopen.SdoCommunicationError:
            rospy.logwarn(f"[{rospy.get_name()}] PantherCAN: SdoCommunicationError occurred while reading wheels velocity (front controller)")
            self._error_handle()

        try:
            wheel_vel[2] = self.rear_controller.sdo["Qry_ABSPEED"][2].raw
            wheel_vel[3] = self.rear_controller.sdo["Qry_ABSPEED"][1].raw
        except canopen.SdoCommunicationError:
            rospy.logwarn(f"[{rospy.get_name()}] PantherCAN: SdoCommunicationError occurred while reading wheels velocity (rear controller)")
            self._error_handle()

        return wheel_vel

    def get_motor_enc_current(self):
        wheel_curr = [0, 0, 0, 0]

        # division by 10 is needed according to documentation
        try:
            wheel_curr[0] = self.front_controller.sdo["Qry_MOTAMPS"][2].raw / 10.0
            wheel_curr[1] = self.front_controller.sdo["Qry_MOTAMPS"][1].raw / 10.0
        except canopen.SdoCommunicationError:
            rospy.logwarn(f"[{rospy.get_name()}] PantherCAN: SdoCommunicationError occurred while reading motor current (front controller)")
            self._error_handle()

        try:
            wheel_curr[2] = self.rear_controller.sdo["Qry_MOTAMPS"][2].raw / 10.0
            wheel_curr[3] = self.rear_controller.sdo["Qry_MOTAMPS"][1].raw / 10.0
        except canopen.SdoCommunicationError:
            rospy.logwarn(f"[{rospy.get_name()}] PantherCAN: SdoCommunicationError occurred while reading motor current (rear controller)")
            self._error_handle()

        return wheel_curr

    def get_battery_data(self):
        battery_data = [0, 0, 0, 0, False] # V_front, I_front, V_rear, I_rear, Error

        # division by 10 is needed according to documentation
        try:
            battery_data[0] = float(self.front_controller.sdo['Qry_VOLTS'][2].raw)/10
            battery_data[1] = float(self.front_controller.sdo['Qry_BATAMPS'][1].raw)/10
        except canopen.SdoCommunicationError:
            rospy.logwarn(f"[{rospy.get_name()}] PantherCAN: SdoCommunicationError occurred while reading battery data (front controller)")
            self._error_handle()
            battery_data[4] = True

        try:
            battery_data[2] = float(self.rear_controller.sdo['Qry_VOLTS'][2].raw)/10
            battery_data[3] = float(self.rear_controller.sdo['Qry_BATAMPS'][1].raw)/10
        except canopen.SdoCommunicationError:
            rospy.logwarn(f"[{rospy.get_name()}] PantherCAN: SdoCommunicationError occurred while reading battery data (rear controller)")
            self._error_handle()
            battery_data[4] = True

        return battery_data

    def read_fault_flags(self):

        fault_flags = [None, None]
        try:
            fault_flags[0] = self.front_controller.sdo["Qry_FLTFLAG"].raw
            fault_flags[1] = self.rear_controller.sdo["Qry_FLTFLAG"].raw
        except canopen.SdoCommunicationError:
            rospy.logwarn(f"[{rospy.get_name()}] PantherCAN: SdoCommunicationError occurred while reading fault flags")
            self._error_handle()

        return fault_flags

    def read_runtime_stat_flag(self):
        runtime_stat_flag = [None, None, None, None]
        try:
            runtime_stat_flag[0] = int.from_bytes(self.front_controller.sdo["Qry_MOTFLAGS"][1].data, "little") 
            runtime_stat_flag[1] = int.from_bytes(self.front_controller.sdo["Qry_MOTFLAGS"][2].data, "little")
            runtime_stat_flag[2] = int.from_bytes(self.rear_controller.sdo["Qry_MOTFLAGS"][1].data, "little")
            runtime_stat_flag[3] = int.from_bytes(self.rear_controller.sdo["Qry_MOTFLAGS"][2].data, "little")
        except canopen.SdoCommunicationError:
            rospy.logwarn(f"[{rospy.get_name()}] PantherCAN: SdoCommunicationError occurred while reading runtime status flag")
            self._error_handle()
        
        return runtime_stat_flag
        
    def _error_handle(self):
        self.error_cnt += 1

        if self.error_cnt >= self.error_max_cnt:
            # !TODO
            #   triger estop when communication failed
            rospy.logerr(f"[{rospy.get_name()}] PantherCAN: huuuuge error...")

    def _turn_on_emergency_stop(self):
        self.front_controller.sdo["Cmd_ESTOP"].raw = 1
        self.rear_controller.sdo["Cmd_ESTOP"].raw = 1

    def _turn_off_emergency_stop(self):
        self.front_controller.sdo["Cmd_MGO"].raw = 1
        self.rear_controller.sdo["Cmd_MGO"].raw = 1