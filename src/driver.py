#!/usr/bin/env python
import math
import rospy
import canopen
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

linear_vel = 0
angular_vel = 0
robot_width = 0
robot_length = 0
wheel_radius = 0
encoder_resolution = 0
L_enc_speed = 0
R_enc_speed = 0
robot_x_pos = 0
robot_y_pos = 0

def euler_to_quaternion(yaw, pitch, roll):

        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]

def cmd_vel_callback(data):
    # forward kinematics

    global linear_vel
    global angular_vel
    global L_enc_speed
    global R_enc_speed
    linear_vel = data.linear.x
    angular_vel = data.angular.z
    L_wheel_lin_speed = float(linear_vel) - (float(angular_vel) * float(robot_width) / 2.0)
    R_wheel_lin_speed = float(linear_vel) + (float(angular_vel) * float(robot_width) / 2.0)
    L_wheel_angular_velocity = float(L_wheel_lin_speed) / float(wheel_radius)
    R_wheel_angular_velocity = float(R_wheel_lin_speed) / float(wheel_radius)
    L_enc_speed = float(encoder_resolution) * float(L_wheel_angular_velocity) / (2 * math.pi)
    R_enc_speed = float(encoder_resolution) * float(R_wheel_angular_velocity) / (2 * math.pi)

    
def panther_driver():
    global robot_width
    global robot_length
    global wheel_radius
    global encoder_resolution
    global robot_x_pos
    global robot_y_pos

    rospy.init_node('~', anonymous=True)
    battery_publisher = rospy.Publisher('battery', BatteryState, queue_size=1)
    battery_msg = BatteryState()

    joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=1)
    joint_state_msg = JointState()
    joint_state_msg.header.frame_id = 'base_link'
    joint_state_msg.name = ['front_left', 'front_right', 'rear_left', 'rear_right']

    pose_publisher = rospy.Publisher('pose', Pose, queue_size=1)
    pose_msg = Pose()
    pose_msg.position.x = 0
    pose_msg.position.y = 0
    pose_msg.position.z = 0
    pose_msg.orientation.x = 0
    pose_msg.orientation.y = 0
    pose_msg.orientation.z = 0
    pose_msg.orientation.w = 1

    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

    can_interface = rospy.get_param('~can_interface', 'panther_can')
    robot_width = rospy.get_param('~robot_width', 0.67)
    robot_length = rospy.get_param('~robot_length', 0.44)
    wheel_radius = rospy.get_param('~wheel_radius', 0.1825)
    encoder_resolution = rospy.get_param('~encoder_resolution', 38400)

    if rospy.has_param('~eds_file'):
        eds_file = rospy.get_param('~eds_file')
    else:
        rospy.logerr("eds_file not defined, can not start CAN interface")
        return

    loop_rate = 10
    rate = rospy.Rate(loop_rate) # 10hz
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

    while not rospy.is_shutdown():
        try:
            front_controller.sdo['Cmd_CANGO'][2].raw = L_enc_speed
            front_controller.sdo['Cmd_CANGO'][1].raw = R_enc_speed
            rear_controller.sdo['Cmd_CANGO'][2].raw = L_enc_speed
            rear_controller.sdo['Cmd_CANGO'][1].raw = R_enc_speed

            battery_msg.voltage = float(front_controller.sdo[0x210D][2].raw)/10
            battery_msg.current = float(front_controller.sdo['Qry_BATAMPS'][1].raw)/10
            battery_publisher.publish(battery_msg)

            #inverse kinematics
            position_FL = front_controller.sdo['Qry_ABCNTR'][2].raw
            position_FR = front_controller.sdo['Qry_ABCNTR'][1].raw
            position_RL = rear_controller.sdo['Qry_ABCNTR'][2].raw
            position_RR = rear_controller.sdo['Qry_ABCNTR'][1].raw

            speed_FL = front_controller.sdo['Qry_ABSPEED'][2].raw
            speed_FR = front_controller.sdo['Qry_ABSPEED'][1].raw
            speed_RL = rear_controller.sdo['Qry_ABSPEED'][2].raw
            speed_RR = rear_controller.sdo['Qry_ABSPEED'][1].raw

            motamps_FL = float(front_controller.sdo['Qry_MOTAMPS'][2].raw)/10
            motamps_FR = float(front_controller.sdo['Qry_MOTAMPS'][1].raw)/10
            motamps_RL = float(rear_controller.sdo['Qry_MOTAMPS'][2].raw)/10
            motamps_RR = float(rear_controller.sdo['Qry_MOTAMPS'][1].raw)/10

            joint_state_msg.position = [position_FL, position_FR, position_RL, position_RR]
            joint_state_msg.velocity = [speed_FL, speed_FR, speed_RL, speed_RR]
            joint_state_msg.effort = [motamps_FL, motamps_FR, motamps_RL, motamps_RR]
            joint_state_publisher.publish(joint_state_msg)

            wheel_L_ang_pos = 3.14 * wheel_radius * (position_FL + position_RL) / encoder_resolution
            wheel_R_ang_pos = 3.14 * wheel_radius * (position_FR + position_RR) / encoder_resolution

            wheel_L_ang_vel = (3.14 * (speed_FL + speed_RL) * wheel_radius / encoder_resolution)
            wheel_R_ang_vel = (3.14 * (speed_FR + speed_RR) * wheel_radius / encoder_resolution)

            robot_angular_pos = (wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / robot_width
            robot_angular_vel = (wheel_R_ang_vel - wheel_L_ang_vel) * wheel_radius / robot_width
            # rospy.loginfo("Wheel angular pos: [%f, %f], rbot angular pos: %f, angular vel: %f", wheel_L_ang_pos, wheel_R_ang_pos, robot_angular_pos, robot_angular_vel)
            robot_x_vel = (wheel_L_ang_vel * wheel_radius + robot_angular_vel * robot_width / 2) * math.cos(robot_angular_pos)
            robot_y_vel = (wheel_R_ang_vel * wheel_radius + robot_angular_vel * robot_width / 2) * math.sin(robot_angular_pos)
            # rospy.loginfo("ENC: [%d, %d]",position_FL, position_FR)
            robot_x_pos = robot_x_pos + robot_x_vel / loop_rate
            robot_y_pos = robot_y_pos + robot_y_vel / loop_rate
            robot_th_pos = robot_th_pos + robot_angular_pos / loop_rate

            qz, qw = euler_to_quaternion(0,0,robot_th_pos)
            # convert robot angular pose to quaternion and publish to pose_msg

            pose_msg.position.x = robot_x_pos
            pose_msg.position.y = robot_y_pos
            pose_msg.orientation.z = qz
            pose_msg.orientation.w = qw
            pose_publisher.publish(pose_msg)
        except:
            rospy.logerr("CAN protocol error")

        rate.sleep()

if __name__ == '__main__':
    try:
        panther_driver()
    except rospy.ROSInterruptException:
        pass