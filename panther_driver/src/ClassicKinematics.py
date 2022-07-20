#!/usr/bin/python3

import math
import rospy
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
import tf2_ros
from PantherKinematics import PantherKinematics

class PantherClassic(PantherKinematics):
    def __init__(self):
        super().__init__()

    def forwardKinematics(self):
        # Classic:
        wheel_front_right = (1/self.wheel_radius) * (self.lin_x +
                                                     (self.robot_width + self.robot_length) * self.ang_z)  # rad/s
        wheel_front_left = (1/self.wheel_radius) * (self.lin_x -
                                                    (self.robot_width + self.robot_length) * self.ang_z)
        wheel_rear_right = (1/self.wheel_radius) * (self.lin_x +
                                                    (self.robot_width + self.robot_length) * self.ang_z)
        wheel_rear_left = (1/self.wheel_radius) * (self.lin_x -
                                                   (self.robot_width + self.robot_length) * self.ang_z)
        self.FR_enc_speed, self.FL_enc_speed, self.RR_enc_speed, self.RL_enc_speed = self._getMotorSpeed(
            wheel_front_right, wheel_front_left, wheel_rear_right, wheel_rear_left)

    def inverseKinematics(self, wheel_FL_ang_vel, wheel_FR_ang_vel, wheel_RL_ang_vel, wheel_RR_ang_vel, dt_):
        # Classic:
        linear_velocity_x_ = (wheel_FL_ang_vel + wheel_FR_ang_vel +
                              wheel_RL_ang_vel + wheel_RR_ang_vel) * (self.wheel_radius/4)
        linear_velocity_y_ = (-wheel_FL_ang_vel + wheel_FR_ang_vel +
                              wheel_RL_ang_vel - wheel_RR_ang_vel) * (self.wheel_radius/4)
        angular_velocity_z_ = (-wheel_FL_ang_vel + wheel_FR_ang_vel - wheel_RL_ang_vel + wheel_RR_ang_vel) * (
            self.wheel_radius/(4 * (self.robot_width / 2 + self.robot_length / 2)))

        delta_heading = angular_velocity_z_ * dt_  # [radians]
        self.robot_th_pos = self.robot_th_pos + delta_heading
        delta_x = (linear_velocity_x_ * math.cos(self.robot_th_pos) -
                   linear_velocity_y_ * math.sin(self.robot_th_pos)) * dt_  # [m]
        delta_y = (linear_velocity_x_ * math.sin(self.robot_th_pos) +
                   linear_velocity_y_ * math.cos(self.robot_th_pos)) * dt_  # [m]
        self.robot_x_pos = self.robot_x_pos + delta_x
        self.robot_y_pos = self.robot_y_pos + delta_y
        return self.robot_x_pos, self.robot_y_pos, self.robot_th_pos
