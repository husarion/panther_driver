#!/usr/bin/env python
import math
import rospy
import canopen
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
import tf2_ros

class PantherKinematics():
    def __init__(self):
        self.lin_x = 0
        self.lin_y = 0
        self.ang_z = 0
        self.robot_width = 0
        self.robot_length = 0
        self.wheel_radius = 0 #Distance of the wheel center, to the roller center
        self.encoder_resolution = 0
        self.FR_enc_speed = 0 #front right encoder speed
        self.FL_enc_speed = 0
        self.RR_enc_speed = 0
        self.RL_enc_speed = 0
        self.robot_x_pos = 0
        self.robot_y_pos = 0
        self.robot_th_pos = 0
        self.power_factor = 0
        self.scale_factor_x = 0.25
        self.scale_factor_y = 0.25 
        self.scale_factor_th = 0.125 
        self.cmd_vel_command_time = rospy.Time()

    def cmd_vel_callback(self,data):
        # forward kinematics
        self.cmd_vel_command_time = rospy.Time.now()
        self.lin_x = data.linear.x * self.scale_factor_x # m/s
        self.lin_y = data.linear.y * self.scale_factor_y # m/s
        self.ang_z = data.angular.z * self.scale_factor_th # rad/s
        self.forwardKinematics()

    def forwardKinematics(self, x_data, y_data, th_data):
        print("not implemented")
        return 1

    def inverseKinematics(self, wheel_FL_ang_vel, wheel_FR_ang_vel, wheel_RL_ang_vel, wheel_RR_ang_vel):
        print("not implemented")
        return 1 

    def _getMotorSpeed(self, wheel_front_right, wheel_front_left, wheel_rear_right, wheel_rear_left):
        
        FR_enc_speed = self.power_factor * float(wheel_front_right) * float(self.encoder_resolution) / (2 * math.pi) # motor power / cmd cango
        FL_enc_speed = self.power_factor * float(wheel_front_left) * float(self.encoder_resolution) / (2 * math.pi) # motor power / cmd cango
        RR_enc_speed = self.power_factor * float(wheel_rear_right) * float(self.encoder_resolution) / (2 * math.pi) # motor power / cmd cango
        RL_enc_speed = self.power_factor * float(wheel_rear_left) * float(self.encoder_resolution) / (2 * math.pi) # motor power / cmd cango

        #limit max power to 1000
        if abs(FR_enc_speed) > 1000:
            FR_enc_speed = 1000 * FR_enc_speed / abs(FR_enc_speed)
        if abs(FL_enc_speed) > 1000:
            FL_enc_speed = 1000 * FL_enc_speed / abs(FL_enc_speed)
        if abs(RR_enc_speed) > 1000:
            RR_enc_speed = 1000 * RR_enc_speed / abs(RR_enc_speed)
        if abs(RL_enc_speed) > 1000:
            RL_enc_speed = 1000 * RL_enc_speed / abs(RL_enc_speed)

        return FR_enc_speed, FL_enc_speed , RR_enc_speed, RL_enc_speed