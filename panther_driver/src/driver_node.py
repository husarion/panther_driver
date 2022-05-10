#!/usr/bin/python3

import math
import rospy
import canopen
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from PantherKinematics import PantherKinematics
from ClassicKinematics import PantherClassic
from MecanumKinematics import PantherMecanum
from MixedKinematics import PantherMix


def factory(kinematics_type=0):
    if kinematics_type == "classic":
        rospy.loginfo("initializing classic kinematics")
        return PantherClassic()
    elif kinematics_type == "mecanum":
        rospy.loginfo("initializing mecanum kinematics")
        return PantherMecanum()
    elif kinematics_type == "mix":
        rospy.loginfo("initializing mixed kinematics")
        return PantherMix()
    else:
        rospy.logerr(
            "Unrecognized kinematics type, provide classic, mecanum or mix as rosparam [~wheel_type]")


def eulerToQuaternion(yaw, pitch, roll):
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

    rospy.init_node('~', anonymous=False)
    kinematics_type = rospy.get_param('~wheel_type', "classic")
    odom_frame = rospy.get_param('~odom_frame', "odom")
    base_link_frame = rospy.get_param('~base_link_frame', "base_link")
    publish_tf = rospy.get_param('~publish_tf', True)
    publish_odometry = rospy.get_param('~publish_odometry', False)
    publish_pose = rospy.get_param('~publish_pose', True)

    RK = factory(kinematics_type)
    br = tf2_ros.TransformBroadcaster()
    tf = TransformStamped()
    battery_publisher = rospy.Publisher('battery', BatteryState, queue_size=1)
    battery_msg = BatteryState()

    joint_state_publisher = rospy.Publisher(
        'joint_states', JointState, queue_size=1)
    joint_state_msg = JointState()
    joint_state_msg.header.frame_id = base_link_frame
    joint_state_msg.name = ['front_left',
                            'front_right', 'rear_left', 'rear_right']

    if publish_pose == True:
        pose_publisher = rospy.Publisher('pose', Pose, queue_size=1)
        pose_msg = Pose()
        pose_msg.position.x = 0
        pose_msg.position.y = 0
        pose_msg.position.z = 0
        pose_msg.orientation.x = 0
        pose_msg.orientation.y = 0
        pose_msg.orientation.z = 0
        pose_msg.orientation.w = 1

    if publish_odometry == True:
        odom_publisher = rospy.Publisher('odom/wheel', Odometry, queue_size=1)
        odom_msg = Odometry()

    rospy.Subscriber("/cmd_vel", Twist, RK.cmdVelCallback, queue_size=1)

    can_interface = rospy.get_param('~can_interface', 'panther_can')
    RK.robot_width = rospy.get_param('~robot_width', 0.682)
    RK.robot_length = rospy.get_param('~robot_length', 0.44)
    RK.wheel_radius = rospy.get_param('~wheel_radius', 0.1825)
    RK.encoder_resolution = rospy.get_param('~encoder_resolution', 30*400*4)
    RK.power_factor = rospy.get_param('~power_factor', 0.04166667)

    if rospy.has_param('~eds_file'):
        eds_file = rospy.get_param('~eds_file')
    else:
        rospy.logerr("eds_file not defined, can not start CAN interface")
        return

    loop_rate = 50
    rate = rospy.Rate(loop_rate)
    rospy.loginfo("Start with creating a network representing one CAN bus")
    network = canopen.Network()
    rospy.loginfo("Add some nodes with corresponding Object Dictionaries")
    front_controller = canopen.RemoteNode(1, eds_file)
    rear_controller = canopen.RemoteNode(2, eds_file)
    network.add_node(front_controller)
    network.add_node(rear_controller)
    rospy.loginfo("Connect to the CAN bus")
    network.connect(channel=can_interface, bustype='socketcan')

    # rospy.loginfo("Reset encoders")
    # front_controller.sdo['Cmd_SENCNTR'][1].raw = 0
    # front_controller.sdo['Cmd_SENCNTR'][2].raw = 0

    robot_x_pos = 0.0
    robot_y_pos = 0.0
    robot_th_pos = 0.0

    wheel_FL_ang_pos_last = 0.0
    wheel_FR_ang_pos_last = 0.0
    wheel_RL_ang_pos_last = 0.0
    wheel_RR_ang_pos_last = 0.0
    last_time = rospy.Time.now()

    # VARIABLE FOR ERROR TRACKING
    err_count = 0

    while not rospy.is_shutdown():
        try:
            rospy.get_master().getPid()
        except:
            rospy.logerr("Error getting master")
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
                front_controller.sdo['Cmd_CANGO'][2].raw = RK.FL_enc_speed
            except:
                rospy.logwarn("Error while writing to front left Cmd_CANGO")
            try:
                front_controller.sdo['Cmd_CANGO'][1].raw = RK.FR_enc_speed
            except:
                rospy.logwarn("Error while writing to front right Cmd_CANGO")
            try:
                rear_controller.sdo['Cmd_CANGO'][2].raw = RK.RL_enc_speed
            except:
                rospy.logwarn("Error while writing to rear left Cmd_CANGO")
            try:
                rear_controller.sdo['Cmd_CANGO'][1].raw = RK.RR_enc_speed
            except:
                rospy.logwarn("Error while writing to rear right Cmd_CANGO")

            try:
                battery_msg.voltage = float(
                    front_controller.sdo[0x210D][2].raw)/10
                battery_msg.current = float(
                    front_controller.sdo['Qry_BATAMPS'][1].raw)/10
                battery_publisher.publish(battery_msg)
            except:
                rospy.logwarn("Error getting battery data")

            # inverse kinematics
            try:
                position_FL = front_controller.sdo['Qry_ABCNTR'][2].raw
            except:
                rospy.logwarn("Error reading front left controller sdo")
            try:
                position_FR = front_controller.sdo['Qry_ABCNTR'][1].raw
            except:
                rospy.logwarn("Error reading front right controller sdo")
            try:
                position_RL = rear_controller.sdo['Qry_ABCNTR'][2].raw
            except:
                rospy.logwarn("Error reading rear left controller sdo")
            try:  
                position_RR = rear_controller.sdo['Qry_ABCNTR'][1].raw
            except:
                rospy.logwarn("Error reading rear right controller sdo")

            # position_FL / RK.encoder_resolution - > full wheel rotations
            wheel_FL_ang_pos = 2 * math.pi * position_FL / RK.encoder_resolution  # radians
            wheel_FR_ang_pos = 2 * math.pi * position_FR / RK.encoder_resolution
            wheel_RL_ang_pos = 2 * math.pi * position_RL / RK.encoder_resolution
            wheel_RR_ang_pos = 2 * math.pi * position_RR / RK.encoder_resolution

            wheel_FL_ang_vel = (wheel_FL_ang_pos -
                                wheel_FL_ang_pos_last) * dt_  # rad/s
            wheel_FR_ang_vel = (wheel_FR_ang_pos - wheel_FR_ang_pos_last) * dt_
            wheel_RL_ang_vel = (wheel_RL_ang_pos - wheel_RL_ang_pos_last) * dt_
            wheel_RR_ang_vel = (wheel_RR_ang_pos - wheel_RR_ang_pos_last) * dt_

            wheel_FL_ang_pos_last = wheel_FL_ang_pos
            wheel_FR_ang_pos_last = wheel_FR_ang_pos
            wheel_RL_ang_pos_last = wheel_RL_ang_pos
            wheel_RR_ang_pos_last = wheel_RR_ang_pos

            try:
                robot_x_pos, robot_y_pos, robot_th_pos = RK.inverseKinematics(
                    wheel_FL_ang_vel, wheel_FR_ang_vel, wheel_RL_ang_vel, wheel_RR_ang_vel, dt_)
            except:
                rospy.logwarn("Couldn't get robot pose")
            # rospy.loginfo("Robot pos: [x, y, th]: [%0.3f, %0.3f, %0.3f]" % (robot_x_pos, robot_y_pos, robot_th_pos*180/3.14))
            RK.wheels_angular_velocity = [
                wheel_FL_ang_vel, wheel_FR_ang_vel, wheel_RR_ang_vel, wheel_RL_ang_vel]
            qx, qy, qz, qw = eulerToQuaternion(robot_th_pos, 0, 0)

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
                odom_msg.header.frame_id = odom_frame
                odom_msg.header.stamp = now
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
            err_count+=1
            if err_count >= 10:
                return

        rate.sleep()


if __name__ == '__main__':
    try:
        driverNode()
    except rospy.ROSInterruptException:
        pass
