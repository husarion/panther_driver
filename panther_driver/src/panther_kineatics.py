#!/usr/bin/python3

import math
from abc import abstractmethod

import rospy

from geometry_msgs.msg import Twist


class PantherKinematics:
    def __init__(self):
        self.lin_x = 0
        self.lin_y = 0
        self.ang_z = 0
        self.robot_width = 0
        self.robot_length = 0
        self.wheel_radius = 0  # Distance of the wheel center, to the roller center
        self.encoder_resolution = 0
        self.gear_ratio = 0
        self.FR_enc_speed = 0  # front right encoder speed
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
        self.cmd_vel_command_time = rospy.Time.now()

    @abstractmethod
    def forward_kinematics(self, x_data, y_data, th_data) -> None:
        pass

    @abstractmethod
    def inverse_kinematics(self, wheel_FL_ang_vel, wheel_FR_ang_vel, wheel_RL_ang_vel, wheel_RR_ang_vel) -> float:
        pass

    def _get_motor_speed(self, wheel_front_right, wheel_front_left, wheel_rear_right, wheel_rear_left):
        
        _const_factor = self.power_factor * float(self.encoder_resolution * self.gear_ratio) / (2 * math.pi)

        FR_enc_speed = _const_factor * float(wheel_front_right)
        FL_enc_speed = _const_factor * float(wheel_front_left)
        RR_enc_speed = _const_factor * float(wheel_rear_right)
        RL_enc_speed = _const_factor * float(wheel_rear_left)

        # limit max power to 1000
        if abs(FR_enc_speed) > 1000:
            FR_enc_speed = 1000 * FR_enc_speed / abs(FR_enc_speed)
        if abs(FL_enc_speed) > 1000:
            FL_enc_speed = 1000 * FL_enc_speed / abs(FL_enc_speed)
        if abs(RR_enc_speed) > 1000:
            RR_enc_speed = 1000 * RR_enc_speed / abs(RR_enc_speed)
        if abs(RL_enc_speed) > 1000:
            RL_enc_speed = 1000 * RL_enc_speed / abs(RL_enc_speed)

        return FR_enc_speed, FL_enc_speed, RR_enc_speed, RL_enc_speed


class PantherDifferential(PantherKinematics):
    def __init__(self):
        super().__init__()

    def forward_kinematics(self, data: Twist):
        self.cmd_vel_command_time = rospy.Time.now()
        self.lin_x = data.linear.x * self.scale_factor_x  # [m/s]
        self.lin_y = data.linear.y * self.scale_factor_y  # [m/s]
        self.ang_z = data.angular.z * self.scale_factor_th  # [rad/s]

        wheel_front_right_ang_vel = wheel_rear_right_ang_vel = (
            1 / self.wheel_radius
        ) * (self.lin_x + (self.robot_width + self.robot_length) * self.ang_z)

        wheel_front_left_ang_vel = wheel_rear_left_ang_vel = (1 / self.wheel_radius) * (
            self.lin_x - (self.robot_width + self.robot_length) * self.ang_z
        )

        self.FR_enc_speed, self.FL_enc_speed, self.RR_enc_speed, self.RL_enc_speed = self._get_motor_speed(
                wheel_front_right_ang_vel,
                wheel_front_left_ang_vel,
                wheel_rear_right_ang_vel,
                wheel_rear_left_ang_vel,
            )

    def inverse_kinematics(self, wheel_FL_ang_vel, wheel_FR_ang_vel, wheel_RL_ang_vel, wheel_RR_ang_vel, dt_):
        _linear_velocity_x = (
            wheel_FL_ang_vel + wheel_FR_ang_vel + wheel_RL_ang_vel + wheel_RR_ang_vel
        ) * (self.wheel_radius / 4)

        _linear_velocity_y = (
            -wheel_FL_ang_vel + wheel_FR_ang_vel + wheel_RL_ang_vel - wheel_RR_ang_vel
        ) * (self.wheel_radius / 4)

        _angular_velocity_z = (
            -wheel_FL_ang_vel + wheel_FR_ang_vel - wheel_RL_ang_vel + wheel_RR_ang_vel
        ) * (self.wheel_radius / (4 * (self.robot_width / 2 + self.robot_length / 2)))

        delta_heading = _angular_velocity_z * dt_  # [radians]
        self.robot_th_pos = self.robot_th_pos + delta_heading

        delta_x = (
            _linear_velocity_x * math.cos(self.robot_th_pos) - _linear_velocity_y * math.sin(self.robot_th_pos)
        ) * dt_  # [m]

        delta_y = (
            _linear_velocity_x * math.sin(self.robot_th_pos) + _linear_velocity_y * math.cos(self.robot_th_pos)
        ) * dt_  # [m]

        self.robot_x_pos = self.robot_x_pos + delta_x
        self.robot_y_pos = self.robot_y_pos + delta_y

        return self.robot_x_pos, self.robot_y_pos, self.robot_th_pos


class PantherMecanum(PantherKinematics):
    def __init__(self):
        super().__init__()

    def forward_kinematics(self, data: Twist):
        self.cmd_vel_command_time = rospy.Time.now()
        self.lin_x = data.linear.x * self.scale_factor_x  # [m/s]
        self.lin_y = data.linear.y * self.scale_factor_y  # [m/s]
        self.ang_z = data.angular.z * self.scale_factor_th  # [rad/s]
        
        wheel_front_right_ang_vel = (1 / self.wheel_radius) * (
            self.lin_x + self.lin_y + (self.robot_width + self.robot_length) * self.ang_z
        )
        wheel_front_left_ang_vel = (1 / self.wheel_radius) * (
            self.lin_x - self.lin_y - (self.robot_width + self.robot_length) * self.ang_z
        )
        wheel_rear_right_ang_vel = (1 / self.wheel_radius) * (
            self.lin_x - self.lin_y + (self.robot_width + self.robot_length) * self.ang_z
        )
        wheel_rear_left_ang_vel = (1 / self.wheel_radius) * (
            self.lin_x + self.lin_y - (self.robot_width + self.robot_length) * self.ang_z
        )

        self.FR_enc_speed, self.FL_enc_speed, self.RR_enc_speed, self.RL_enc_speed = self._get_motor_speed(
                wheel_front_right_ang_vel,
                wheel_front_left_ang_vel,
                wheel_rear_right_ang_vel,
                wheel_rear_left_ang_vel,
            )

    def inverse_kinematics(self, wheel_FL_ang_vel, wheel_FR_ang_vel, wheel_RL_ang_vel, wheel_RR_ang_vel, dt_):
        linear_velocity_x_ = (
            wheel_FL_ang_vel + wheel_FR_ang_vel + wheel_RL_ang_vel + wheel_RR_ang_vel
        ) * (self.wheel_radius / 4)

        linear_velocity_y_ = (
            -wheel_FL_ang_vel + wheel_FR_ang_vel + wheel_RL_ang_vel - wheel_RR_ang_vel
        ) * (self.wheel_radius / 4)

        angular_velocity_z_ = (
            -wheel_FL_ang_vel + wheel_FR_ang_vel - wheel_RL_ang_vel + wheel_RR_ang_vel
        ) * (self.wheel_radius / (4 * (self.robot_width / 2 + self.robot_length / 2)))

        delta_heading = angular_velocity_z_ * dt_  # [radians]
        self.robot_th_pos = self.robot_th_pos + delta_heading

        delta_x = (
            linear_velocity_x_ * math.cos(self.robot_th_pos)
            - linear_velocity_y_ * math.sin(self.robot_th_pos)
        ) * dt_  # [m]

        delta_y = (
            linear_velocity_x_ * math.sin(self.robot_th_pos)
            + linear_velocity_y_ * math.cos(self.robot_th_pos)
        ) * dt_  # [m]

        self.robot_x_pos = self.robot_x_pos + delta_x
        self.robot_y_pos = self.robot_y_pos + delta_y

        return self.robot_x_pos, self.robot_y_pos, self.robot_th_pos
