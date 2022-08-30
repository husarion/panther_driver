#!/usr/bin/python3

import canopen
import math

import rospy
import tf2_ros
from numpy import NaN

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from panther_msgs.msg import BatteryDriver

from ClassicKinematics import PantherClassic
from MecanumKinematics import PantherMecanum
from MixedKinematics import PantherMix


def sign(x):
    x_sign = bool(x > 0) - bool(x < 0)
    return x_sign if x_sign != 0 else 1

def factory(kinematics_type=0):
    if kinematics_type == "classic":
        rospy.loginfo(f"[{rospy.get_name()}] initializing classic kinematics")
        return PantherClassic()
    elif kinematics_type == "mecanum":
        rospy.loginfo(f"[{rospy.get_name()}] initializing mecanum kinematics")
        return PantherMecanum()
    elif kinematics_type == "mix":
        rospy.loginfo(f"[{rospy.get_name()}] initializing mixed kinematics")
        return PantherMix()
    else:
        rospy.logerr(
            f"[{rospy.get_name()}] unrecognized kinematics type, provide classic, mecanum or mix as rosparam [~wheel_type]")


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


def driverNode():

    rospy.init_node("panther_driver", anonymous=False)
    kinematics_type = rospy.get_param("~wheel_type", "classic")
    odom_frame = rospy.get_param("~odom_frame", "odom")
    base_link_frame = rospy.get_param("~base_link_frame", "base_link")
    motor_torque_constant = rospy.get_param("~motor_torque_constant", 2.6149)
    publish_tf = rospy.get_param("~publish_tf", True)
    publish_odometry = rospy.get_param("~publish_odometry", True)
    publish_pose = rospy.get_param("~publish_pose", True)

    RK = factory(kinematics_type)
    br = tf2_ros.TransformBroadcaster()
    tf = TransformStamped()

    battery_driver_publisher = rospy.Publisher('battery_driver', BatteryDriver, queue_size=1)
    battery_driver_msg = BatteryDriver()
    battery_driver_msg.V_front = 0
    battery_driver_msg.V_rear = 0
    battery_driver_msg.I_front = 0
    battery_driver_msg.I_rear = 0
    battery_driver_msg.error = False

    joint_state_publisher = rospy.Publisher(
        "joint_states", JointState, queue_size=1)
    joint_state_msg = JointState()
    joint_state_msg.header.frame_id = base_link_frame
    joint_state_msg.name = [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint"
    ]

    if publish_pose == True:
        pose_publisher = rospy.Publisher("pose", Pose, queue_size=1)
        pose_msg = Pose()
        pose_msg.position.x = 0
        pose_msg.position.y = 0
        pose_msg.position.z = 0
        pose_msg.orientation.x = 0
        pose_msg.orientation.y = 0
        pose_msg.orientation.z = 0
        pose_msg.orientation.w = 1

    if publish_odometry == True:
        odom_publisher = rospy.Publisher("odom/wheel", Odometry, queue_size=1)
        odom_msg = Odometry()

    rospy.Subscriber("/cmd_vel", Twist, RK.cmdVelCallback, queue_size=1)

    can_interface = rospy.get_param("~can_interface", "panther_can")
    RK.robot_width = rospy.get_param("~robot_width", 0.682)
    RK.robot_length = rospy.get_param("~robot_length", 0.44)
    RK.wheel_radius = rospy.get_param("~wheel_radius", 0.1825)
    RK.encoder_resolution = rospy.get_param("~encoder_resolution", 400*4)
    RK.gear_ratio = rospy.get_param("~gear_ratio", 30.08)
    RK.power_factor = rospy.get_param("~power_factor", 0.04166667)

    if rospy.has_param("~eds_file"):
        eds_file = rospy.get_param("~eds_file")
    else:
        rospy.logerr(f"[{rospy.get_name()}] eds_file not defined, can not start CAN interface")
        return

    loop_rate = 20
    rate = rospy.Rate(loop_rate)
    rospy.loginfo(f"[{rospy.get_name()}] Start with creating a network representing one CAN bus")
    network = canopen.Network()
    rospy.loginfo(f"[{rospy.get_name()}] Add some nodes with corresponding Object Dictionaries")
    front_controller = canopen.RemoteNode(1, eds_file)
    rear_controller = canopen.RemoteNode(2, eds_file)
    network.add_node(front_controller)
    network.add_node(rear_controller)
    rospy.loginfo(f"[{rospy.get_name()}] Connect to the CAN bus")
    network.connect(channel=can_interface, bustype="socketcan")

    # rospy.loginfo(f"[{rospy.get_name()}] Reset encoders")
    # front_controller.sdo["Cmd_SENCNTR"][1].raw = 0
    # front_controller.sdo["Cmd_SENCNTR"][2].raw = 0

    robot_x_pos = 0.0
    robot_y_pos = 0.0
    robot_th_pos = 0.0

    wheel_pos = [0.0, 0.0, 0.0, 0.0]
    wheel_vel = [0.0, 0.0, 0.0, 0.0]
    wheel_curr = [0.0, 0.0, 0.0, 0.0]

    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        try:
            rospy.get_master().getPid()
        except:
            rospy.logerr(f"[{rospy.get_name()}] Error getting master")
            exit(1)
        try:
            now = rospy.Time.now()
            dt_ = (now - last_time).to_sec()
            last_time = now

            if((now - RK.cmd_vel_command_time) > rospy.Duration(secs=0.2)):
                RK.FL_enc_speed = 0.0
                RK.FR_enc_speed = 0.0
                RK.RL_enc_speed = 0.0
                RK.RR_enc_speed = 0.0

            try:
                front_controller.sdo["Cmd_CANGO"][2].raw = RK.FL_enc_speed
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error while writing to front left Cmd_CANGO")
            try:
                front_controller.sdo["Cmd_CANGO"][1].raw = RK.FR_enc_speed
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error while writing to front right Cmd_CANGO")
            try:
                rear_controller.sdo["Cmd_CANGO"][2].raw = RK.RL_enc_speed
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error while writing to rear left Cmd_CANGO")
            try:
                rear_controller.sdo["Cmd_CANGO"][1].raw = RK.RR_enc_speed
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error while writing to rear right Cmd_CANGO")

            try:
                front_controller.sdo['Cmd_CANGO'][2].raw = RK.FL_enc_speed
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error reading front left controller sdo")
            try:
                wheel_pos[1] = front_controller.sdo["Qry_ABCNTR"][1].raw
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error reading front right controller sdo")
            try:
                wheel_pos[2] = rear_controller.sdo["Qry_ABCNTR"][2].raw
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error reading rear left controller sdo")
            try:  
                wheel_pos[3] = rear_controller.sdo["Qry_ABCNTR"][1].raw
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error reading rear right controller sdo")

            # query velocity
            try:
                wheel_vel[0] = front_controller.sdo["Qry_ABSPEED"][2].raw
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error reading front left controller sdo")
            try:
                wheel_vel[1] = front_controller.sdo["Qry_ABSPEED"][1].raw
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error reading front right controller sdo")
            try:
                wheel_vel[2] = rear_controller.sdo["Qry_ABSPEED"][2].raw
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error reading rear left controller sdo")
            try:  
                wheel_vel[3] = rear_controller.sdo["Qry_ABSPEED"][1].raw
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error reading rear right controller sdo")

            # query current
            # division by 10 is needed according to documentation
            try:
                wheel_curr[0] = front_controller.sdo["Qry_MOTAMPS"][2].raw / 10.0
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error reading current front left controller sdo")
            try:
                wheel_curr[1] = front_controller.sdo["Qry_MOTAMPS"][1].raw / 10.0
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error reading current front right controller sdo")
            try:
                wheel_curr[2] = rear_controller.sdo["Qry_MOTAMPS"][2].raw / 10.0
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error reading current rear left controller sdo")
            try:  
                wheel_curr[3] = rear_controller.sdo["Qry_MOTAMPS"][1].raw / 10.0
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error reading current rear right controller sdo")

            joint_state_msg.header.stamp = rospy.Time.now()

            # convert tics to rad
            joint_state_msg.position = [(2.0 * math.pi) * (pos / (RK.encoder_resolution * RK.gear_ratio)) for pos in wheel_pos]
            # convert RPM to rad/s
            joint_state_msg.velocity = [(2.0 * math.pi / 60) * (vel / RK.gear_ratio) for vel in wheel_vel]
            # convert to A to Nm
            joint_state_msg.effort = [
               wheel_curr[i] * motor_torque_constant * sign(wheel_vel[i]) for i in range(len(wheel_curr))]

            joint_state_publisher.publish(joint_state_msg)

            # read drivers battery data
            try:
                battery_driver_msg.V_front = float(front_controller.sdo['Qry_VOLTS'][2].raw)/10
                battery_driver_msg.V_rear = float(rear_controller.sdo['Qry_VOLTS'][2].raw)/10
                battery_driver_msg.I_front = float(front_controller.sdo['Qry_BATAMPS'][1].raw)/10
                battery_driver_msg.I_rear = float(rear_controller.sdo['Qry_BATAMPS'][1].raw)/10
                battery_driver_msg.error = False
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Error getting battery data")

            # publish drivers battery data
            battery_driver_publisher.publish(battery_driver_msg)

            try:
                robot_x_pos, robot_y_pos, robot_th_pos = RK.inverseKinematics(
                    *joint_state_msg.velocity, dt_)
            except:
                rospy.logwarn(f"[{rospy.get_name()}] Could not get robot pose")
            qx, qy, qz, qw = euler_to_quaternion(robot_th_pos, 0, 0)

            if publish_pose == True:
                pose_msg.position.x = robot_x_pos
                pose_msg.position.y = robot_y_pos
                pose_msg.orientation.x = qx
                pose_msg.orientation.y = qy
                pose_msg.orientation.z = qz
                pose_msg.orientation.w = qw
                pose_publisher.publish(pose_msg)

            if publish_tf == True:
                tf.header.stamp = rospy.Time.now()
                tf.header.frame_id = odom_frame
                tf.child_frame_id = base_link_frame
                tf.transform.translation.x = robot_x_pos
                tf.transform.translation.y = robot_y_pos
                tf.transform.translation.z = 0.0
                tf.transform.rotation.x = qx
                tf.transform.rotation.y = qy
                tf.transform.rotation.z = qz
                tf.transform.rotation.w = qw
                br.sendTransform(tf)

            if publish_odometry == True:
                odom_msg.header.stamp = rospy.Time.now()
                odom_msg.header.frame_id = odom_frame
                odom_msg.pose.pose.position.x = robot_x_pos
                odom_msg.pose.pose.position.y = robot_y_pos
                odom_msg.pose.pose.orientation.x = qx
                odom_msg.pose.pose.orientation.y = qy
                odom_msg.pose.pose.orientation.z = qz
                odom_msg.pose.pose.orientation.w = qw
                odom_msg.twist.twist.linear.x = 0
                odom_msg.twist.twist.linear.y = 0
                odom_msg.twist.twist.linear.z = 0
                odom_msg.twist.twist.angular.x = 0
                odom_msg.twist.twist.angular.y = 0
                odom_msg.twist.twist.angular.z = 0
                odom_publisher.publish(odom_msg)

        except:
            rospy.logerr("CAN protocol error")

        rate.sleep()


if __name__ == "__main__":
    try:
        driverNode()
    except Exception as e:
        rospy.logerr(f"[{rospy.get_name()}] Error: {e}")
