#!/usr/bin/python3

import math
import time 

import rospy
import tf2_ros

from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from panther_kineatics import PantherDifferential, PantherMecanum
from panther_can import PantherCAN

from panther_msgs.msg import DriverStateArr, FaultFlag, RuntimeError


def euler_to_quaternion(yaw, pitch, roll):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
        math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
        math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
        math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
        math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return [qx, qy, qz, qw]


class PantherDriver(object):
    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=False)

        assert rospy.has_param(
            "~eds_file"
        ), f"[{rospy.get_name()}] eds_file not defined, can not start CAN interface"
        self.eds_file = rospy.get_param("~eds_file")
        self.can_interface = rospy.get_param("~can_interface", "panther_can")
        self.kinematics_type = rospy.get_param("~wheel_type", "differential")
        self.motor_torque_constant = rospy.get_param("~motor_torque_constant", 2.6149)

        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.publish_odom = rospy.get_param("~publish_odometry", True)
        self.publish_pose = rospy.get_param("~publish_pose", True)
        self.publish_joints = rospy.get_param("~publish_joints", True)

        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_link_frame = rospy.get_param("~base_link_frame", "base_link")
        self.wheels_joints_names = [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
        ]

        self.pose_msg = Pose()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_stamped = TransformStamped()
        self.tf_stamped.header.frame_id = self.odom_frame
        self.tf_stamped.child_frame_id = self.base_link_frame
        self.joint_state_msg = JointState()
        self.joint_state_msg.header.frame_id = self.base_link_frame
        self.joint_state_msg.name = self.wheels_joints_names
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame
        self.driver_state_msg = DriverStateArr()

        self.cmd_vel_timeout = 0.2
        self.main_timer_freq = 1./15.           # 15 Hz
        self.driver_state_timer_freq = 1./2.    # 2 Hz
        self.last_time = rospy.Time.now()

        self.estop_triggered = True
        self.estop_triggered_last = True
        self.stop_cmd_vel_cb = True

        self.robot_x_pos = 0.0
        self.robot_y_pos = 0.0
        self.robot_th_pos = 0.0

        self.wheels_ang_pos = [0.0, 0.0, 0.0, 0.0]
        self.wheels_ang_vel = [0.0, 0.0, 0.0, 0.0]
        self.motors_effort = [0.0, 0.0, 0.0, 0.0]

        # -------------------------------
        #   Kinematic type
        # -------------------------------

        assert self.kinematics_type in [
            "differential",
            "mecanum",
        ], f"[{rospy.get_name()}] The kinematics type is incorrect: {self.kinematics_type}"

        if self.kinematics_type == "differential":
            self.panther_kinematics = PantherDifferential()
        else:
            self.panther_kinematics = PantherMecanum()

        self.panther_kinematics.robot_width = rospy.get_param("~robot_width", 0.682)
        self.panther_kinematics.robot_length = rospy.get_param("~robot_length", 0.44)
        self.panther_kinematics.wheel_radius = rospy.get_param("~wheel_radius", 0.1825)
        self.panther_kinematics.encoder_resolution = rospy.get_param(
            "~encoder_resolution", 400 * 4
        )
        self.panther_kinematics.gear_ratio = rospy.get_param("~gear_ratio", 30.08)
        self.panther_kinematics.power_factor = rospy.get_param(
            "~power_factor", 0.04166667
        )

        # -------------------------------
        #   CAN interface
        # -------------------------------

        self.panther_can = PantherCAN(eds_file=self.eds_file, can_interface=self.can_interface)
        time.sleep(4)

        # -------------------------------
        #   Publishers & Subscribers
        # -------------------------------

        self.joint_publisher = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.pose_publisher = rospy.Publisher("pose", Pose, queue_size=1)
        self.odom_publisher = rospy.Publisher("odom/wheel", Odometry, queue_size=1)
        self.driver_state_publisher = rospy.Publisher('panther_driver/state', DriverStateArr, queue_size=1)

        rospy.Subscriber("/panther_hardware/e_stop", Bool, self._estop_cb, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_cb, queue_size=1)

        # -------------------------------
        #   Timers
        # -------------------------------

        self._main_timer = rospy.Timer(
            rospy.Duration(self.main_timer_freq), self._main_timer_callback
        )

        self._driver_state_timer = rospy.Timer(
            rospy.Duration(self.driver_state_timer_freq), self._driver_state_timer_callback
        )

        # -------------------------------
        # -------------------------------

        rospy.loginfo(f"{rospy.get_name()} node started")


    def _main_timer_callback(self, *args) -> None:
        try:
            rospy.get_master().getPid()
        except:
            rospy.logerr(f"[{rospy.get_name()}] Error getting master.")
            exit(1)

        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        if (now - self.panther_kinematics.cmd_vel_command_time) > rospy.Duration(secs=self.cmd_vel_timeout):
            self.panther_kinematics.FL_enc_speed = 0.0
            self.panther_kinematics.FR_enc_speed = 0.0
            self.panther_kinematics.RL_enc_speed = 0.0
            self.panther_kinematics.RR_enc_speed = 0.0
        
        # w8 for panther_can obj avaible
        while self.panther_can.lock:
            time.sleep(0.01)

        self.panther_can.lock = True

        self.panther_can.set_wheels_enc_velocity(
            self.panther_kinematics.FL_enc_speed,
            self.panther_kinematics.FR_enc_speed,
            self.panther_kinematics.RL_enc_speed,
            self.panther_kinematics.RR_enc_speed,
        )
        wheel_enc_pos = self.panther_can.get_wheels_enc_pose()
        wheel_enc_vel = self.panther_can.get_wheels_enc_velocity()
        wheel_enc_curr = self.panther_can.get_motor_enc_current()

        self.panther_can.lock = False

        # convert tics to rad
        self.wheels_ang_pos = [
            (2.0 * math.pi) * (pos / (self.panther_kinematics.encoder_resolution * self.panther_kinematics.gear_ratio))
            for pos in wheel_enc_pos
        ]
        # convert RPM to rad/s
        self.wheels_ang_vel = [
            (2.0 * math.pi / 60) * (vel / self.panther_kinematics.gear_ratio)
            for vel in wheel_enc_vel
        ]
        # convert A to Nm
        self.motors_effort = [
            wheel_enc_curr[i] * self.motor_torque_constant * math.copysign(1, wheel_enc_vel[i])
            for i in range(len(wheel_enc_curr))
        ]

        try:
            self.robot_x_pos, self.robot_y_pos, self.robot_th_pos = \
                self.panther_kinematics.inverse_kinematics(*self.wheels_ang_vel, dt_=dt) 
        except:
            rospy.logwarn(f"[{rospy.get_name()}] Could not get robot pose")

        self.qx, self.qy, self.qz, self.qw = euler_to_quaternion(self.robot_th_pos, 0, 0)

        if self.publish_joints: self._publish_joint_state()
        if self.publish_pose:   self._publish_pose()
        if self.publish_odom:   self._publish_odom()
        if self.publish_tf:     self._publish_tf()


    def _driver_state_timer_callback(self, *args) -> None:
        while self.panther_can.lock:
            time.sleep(0.01)

        self.panther_can.lock = True
        (
            self.driver_state_msg.front.voltage, 
            self.driver_state_msg.front.current, 
            self.driver_state_msg.rear.voltage, 
            self.driver_state_msg.rear.current,
        ) = self.panther_can.get_battery_data()

        roboteq_fault_flags = self.panther_can.read_fault_flags()
        roboteq_runtime_flags = self.panther_can.read_runtime_stat_flag()
        self.panther_can.lock = False

        self.driver_state_msg.front.fault_flag = self._decode_fault_flag(roboteq_fault_flags[0])
        self.driver_state_msg.rear.fault_flag = self._decode_fault_flag(roboteq_fault_flags[1])

        self.driver_state_msg.front.right_mot_run_err = \
            self._decode_runtime_flag(roboteq_runtime_flags[0]) 
        self.driver_state_msg.front.left_mot_run_err = \
            self._decode_runtime_flag(roboteq_runtime_flags[1]) 
        self.driver_state_msg.rear.right_mot_run_err = \
            self._decode_runtime_flag(roboteq_runtime_flags[2]) 
        self.driver_state_msg.rear.left_mot_run_err = \
            self._decode_runtime_flag(roboteq_runtime_flags[3]) 

        self.driver_state_publisher.publish(self.driver_state_msg)
    

    def _decode_fault_flag(self, flag_val: int) -> FaultFlag:
        # For more info see 272-roboteq-controllers-user-manual-v21 p. 246
        #   https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/
        ref_flag_val = 0b00000001

        msg = FaultFlag()
        msg.can_net_err = self.panther_can.can_net_err
        msg.overheat = bool(flag_val & ref_flag_val << 0)
        msg.overvoltage = bool(flag_val & ref_flag_val << 1)
        msg.undervoltage = bool(flag_val & ref_flag_val << 2)
        msg.short_circuit = bool(flag_val & ref_flag_val << 3)
        msg.emergency_stop = bool(flag_val & ref_flag_val << 4)
        msg.motor_or_sensor_setup_fault = bool(flag_val & ref_flag_val << 5)
        msg.mosfet_failure = bool(flag_val & ref_flag_val << 6)
        msg.default_config_loaded_at_startup = bool(flag_val & ref_flag_val << 7)

        return msg

    def _decode_runtime_flag(self, flag_val: int) -> RuntimeError:
        # For more info see 272-roboteq-controllers-user-manual-v21 p. 247
        #   https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/
        ref_flag_val = 0b00000001

        msg = RuntimeError()
        msg.amps_limit_active = bool(flag_val & ref_flag_val << 0)
        msg.motor_stall = bool(flag_val & ref_flag_val << 1)
        msg.loop_error = bool(flag_val & ref_flag_val << 2)
        msg.safety_stop_active = bool(flag_val & ref_flag_val << 3)
        msg.forward_limit_triggered = bool(flag_val & ref_flag_val << 4)
        msg.reverse_limit_triggered = bool(flag_val & ref_flag_val << 5)
        msg.amps_trigger_activated = bool(flag_val & ref_flag_val << 6)

        return msg

    def _estop_cb(self, data) -> None:
        self.estop_triggered_last = self.estop_triggered
        self.estop_triggered = data.data
        
        # If the safety stop is disabled, the driver ignores messages on the cmd_vel topic until there are
        # only zeros in the Twist message. All this is done so that the robot does not move without 
        # the knowledge of the user
        if not self.estop_triggered and (self.estop_triggered_last != self.estop_triggered):
            self.stop_cmd_vel_cb = True

    def _cmd_vel_cb(self, data) -> None:
        if not self.stop_cmd_vel_cb:
            self.panther_kinematics.forward_kinematics(data)
        elif all(v == 0.0 for v in [data.linear.x, data.linear.y, data.angular.z]):
            self.stop_cmd_vel_cb = False
        else:
            self.panther_kinematics.forward_kinematics(Twist())

    def _publish_joint_state(self) -> None:
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_msg.position = self.wheels_ang_pos
        self.joint_state_msg.velocity = self.wheels_ang_vel
        self.joint_state_msg.effort = self.motors_effort
        self.joint_publisher.publish(self.joint_state_msg)

    def _publish_pose(self) -> None:
        self.pose_msg.position.x = self.robot_x_pos
        self.pose_msg.position.y = self.robot_y_pos
        self.pose_msg.orientation.x = self.qx
        self.pose_msg.orientation.y = self.qy
        self.pose_msg.orientation.z = self.qz
        self.pose_msg.orientation.w = self.qw
        self.pose_publisher.publish(self.pose_msg)

    def _publish_tf(self) -> None:
        self.tf_stamped.header.stamp = rospy.Time.now()
        self.tf_stamped.transform.translation.x = self.robot_x_pos
        self.tf_stamped.transform.translation.y = self.robot_y_pos
        self.tf_stamped.transform.translation.z = 0.0
        self.tf_stamped.transform.rotation.x = self.qx
        self.tf_stamped.transform.rotation.y = self.qy
        self.tf_stamped.transform.rotation.z = self.qz
        self.tf_stamped.transform.rotation.w = self.qw
        self.tf_broadcaster.sendTransform(self.tf_stamped)

    def _publish_odom(self) -> None:
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.pose.pose.position.x = self.robot_x_pos
        self.odom_msg.pose.pose.position.y = self.robot_y_pos
        self.odom_msg.pose.pose.orientation.x = self.qx
        self.odom_msg.pose.pose.orientation.y = self.qy
        self.odom_msg.pose.pose.orientation.z = self.qz
        self.odom_msg.pose.pose.orientation.w = self.qw
        self.odom_publisher.publish(self.odom_msg)
        

def main():
    panther_driver_node = PantherDriver("panther_driver")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
