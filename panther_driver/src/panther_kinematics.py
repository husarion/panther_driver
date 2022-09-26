#!/usr/bin/python3

import math
from numpy import clip
from abc import abstractmethod

import rospy

from geometry_msgs.msg import Twist


class PantherKinematics:
    def __init__(self) -> None:
        self.robot_width = 0.0
        self.robot_length = 0.0
        self.wheel_radius = 0.0  # Distance of the wheel center, to the roller center
        self.encoder_resolution = None
        self.gear_ratio = None
        self.power_factor = None
        self._wheels_enc_speed = [0.0, 0.0, 0.0, 0.0]
        self.cmd_vel_command_time = rospy.Time.now()

        self._lin_x = 0.0
        self._lin_y = 0.0
        self._ang_z = 0.0
        self._robot_x_pos = 0.0
        self._robot_y_pos = 0.0
        self._robot_th_pos = 0.0
        self._max_speed = 950.0
        self._scale_factor_x = 0.25
        self._scale_factor_y = 0.25
        self._scale_factor_th = 0.125
        self._const_factor = None
        
    @abstractmethod
    def inverse_kinematics(self, data: Twist) -> None:
        pass

    @abstractmethod
    def forward_kinematics(self, fl_ang_vel, fr_ang_vel, rl_ang_vel, rr_ang_vel, dt_) -> float:
        pass

    def _get_motor_speed(self, wheel_fl_ang_vel, wheel_fr_ang_vel, wheel_rl_ang_vel, wheel_rr_ang_vel) -> list:
        # makes sure the rosparams are set correctly and calculates _const_factor only once
        if self.power_factor is not None and self._const_factor is None:
            self._const_factor = \
                self.power_factor * float(self.encoder_resolution * self.gear_ratio) / (2.0 * math.pi)
        elif self._const_factor is None:
            return 0.0, 0.0, 0.0, 0.0
        
        # limit max power to 1000
        fl_enc_speed = clip(self._const_factor * float(wheel_fl_ang_vel), -self._max_speed, self._max_speed)
        fr_enc_speed = clip(self._const_factor * float(wheel_fr_ang_vel), -self._max_speed, self._max_speed)
        rl_enc_speed = clip(self._const_factor * float(wheel_rl_ang_vel), -self._max_speed, self._max_speed)
        rr_enc_speed = clip(self._const_factor * float(wheel_rr_ang_vel), -self._max_speed, self._max_speed)

        return [fl_enc_speed, fr_enc_speed, rl_enc_speed, rr_enc_speed]


class PantherDifferential(PantherKinematics):
    def __init__(self) -> None:
        super().__init__()

    def inverse_kinematics(self, data: Twist) -> None:
        self.cmd_vel_command_time = rospy.Time.now()
        self._lin_x = data.linear.x * self._scale_factor_x  # [m/s]
        self._lin_y = data.linear.y * self._scale_factor_y  # [m/s]
        self._ang_z = data.angular.z * self._scale_factor_th  # [rad/s]

        wheel_front_right_ang_vel = wheel_rear_right_ang_vel = (1.0 / self.wheel_radius) * (
            self._lin_x + (self.robot_width + self.robot_length) * self._ang_z
        )
        wheel_front_left_ang_vel = wheel_rear_left_ang_vel = (1.0 / self.wheel_radius) * (
            self._lin_x - (self.robot_width + self.robot_length) * self._ang_z
        )

        self._wheels_enc_speed = self._get_motor_speed(
                wheel_front_left_ang_vel,
                wheel_front_right_ang_vel,
                wheel_rear_left_ang_vel,
                wheel_rear_right_ang_vel,
            )

    def forward_kinematics(self, fl_ang_vel, fr_ang_vel, rl_ang_vel, rr_ang_vel, dt_) -> float:
        linear_velocity_x = (
            fl_ang_vel + fr_ang_vel + rl_ang_vel + rr_ang_vel
        ) * (self.wheel_radius / 4.0)

        linear_velocity_y = (
            -fl_ang_vel + fr_ang_vel + rl_ang_vel - rr_ang_vel
        ) * (self.wheel_radius / 4.0)

        angular_velocity_z = (
            -fl_ang_vel + fr_ang_vel - rl_ang_vel + rr_ang_vel
        ) * (self.wheel_radius / (4.0 * (self.robot_width / 2.0 + self.robot_length / 2)))

        delta_heading = angular_velocity_z * dt_  # [radians]
        self._robot_th_pos = self._robot_th_pos + delta_heading

        delta_x = (
            linear_velocity_x * math.cos(self._robot_th_pos) - linear_velocity_y * math.sin(self._robot_th_pos)
        ) * dt_  # [m]

        delta_y = (
            linear_velocity_x * math.sin(self._robot_th_pos) + linear_velocity_y * math.cos(self._robot_th_pos)
        ) * dt_  # [m]

        self._robot_x_pos = self._robot_x_pos + delta_x
        self._robot_y_pos = self._robot_y_pos + delta_y

        return self._robot_x_pos, self._robot_y_pos, self._robot_th_pos


class PantherMecanum(PantherKinematics):
    def __init__(self) -> None:
        super().__init__()

    def inverse_kinematics(self, data: Twist) -> None:
        self.cmd_vel_command_time = rospy.Time.now()
        self._lin_x = data.linear.x * self._scale_factor_x  # [m/s]
        self._lin_y = data.linear.y * self._scale_factor_y  # [m/s]
        self._ang_z = data.angular.z * self._scale_factor_th  # [rad/s]
        
        wheel_front_right_ang_vel = (1.0 / self.wheel_radius) * (
            self._lin_x + self._lin_y + (self.robot_width + self.robot_length) * self._ang_z
        )
        wheel_front_left_ang_vel = (1.0 / self.wheel_radius) * (
            self._lin_x - self._lin_y - (self.robot_width + self.robot_length) * self._ang_z
        )
        wheel_rear_right_ang_vel = (1.0 / self.wheel_radius) * (
            self._lin_x - self._lin_y + (self.robot_width + self.robot_length) * self._ang_z
        )
        wheel_rear_left_ang_vel = (1.0 / self.wheel_radius) * (
            self._lin_x + self._lin_y - (self.robot_width + self.robot_length) * self._ang_z
        )

        self._wheels_enc_speed = self._get_motor_speed(
                wheel_front_left_ang_vel,
                wheel_front_right_ang_vel,
                wheel_rear_left_ang_vel,
                wheel_rear_right_ang_vel,
            )

    def forward_kinematics(self, fl_ang_vel, fr_ang_vel, rl_ang_vel, rr_ang_vel, dt_) -> float:
        linear_velocity_x = (
            fl_ang_vel + fr_ang_vel + rl_ang_vel + rr_ang_vel
        ) * (self.wheel_radius / 4.0)

        linear_velocity_y = (
            -fl_ang_vel + fr_ang_vel + rl_ang_vel - rr_ang_vel
        ) * (self.wheel_radius / 4.0)

        angular_velocity_z = (
            -fl_ang_vel + fr_ang_vel - rl_ang_vel + rr_ang_vel
        ) * (self.wheel_radius / (4.0 * (self.robot_width / 2.0 + self.robot_length / 2.0)))

        delta_heading = angular_velocity_z * dt_  # [radians]
        self._robot_th_pos = self._robot_th_pos + delta_heading

        delta_x = (
            linear_velocity_x * math.cos(self._robot_th_pos)
            - linear_velocity_y * math.sin(self._robot_th_pos)
        ) * dt_  # [m]

        delta_y = (
            linear_velocity_x * math.sin(self._robot_th_pos)
            + linear_velocity_y * math.cos(self._robot_th_pos)
        ) * dt_  # [m]

        self._robot_x_pos = self._robot_x_pos + delta_x
        self._robot_y_pos = self._robot_y_pos + delta_y

        return self._robot_x_pos, self._robot_y_pos, self._robot_th_pos
