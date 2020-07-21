#!/usr/bin/env python
import math
import rospy
import canopen
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
class RobotKinematics():
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
        # self.roller_radius = 0


def euler_to_quaternion(yaw, pitch, roll):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return [qx, qy, qz, qw]  


RK = RobotKinematics()

def cmd_vel_callback(data):
    # forward kinematics
    global RK
    RK.lin_x = data.linear.x # m/s
    RK.lin_y = data.linear.y
    RK.ang_z = data.angular.z # rad/s

    wheel_front_right = (1/RK.wheel_radius) * (RK.lin_x + RK.lin_y + (RK.robot_width + RK.robot_length)*RK.ang_z) # rad/s
    wheel_front_left = (1/RK.wheel_radius) * (RK.lin_x - RK.lin_y - (RK.robot_width + RK.robot_length)*RK.ang_z)  
    wheel_rear_right = (1/RK.wheel_radius) * (RK.lin_x - RK.lin_y + (RK.robot_width + RK.robot_length)*RK.ang_z)
    wheel_rear_left = (1/RK.wheel_radius) * (RK.lin_x + RK.lin_y - (RK.robot_width + RK.robot_length)*RK.ang_z)
    

    RK.FR_enc_speed = RK.power_factor * float(wheel_front_right) * float(RK.encoder_resolution) / (2 * math.pi) # motor power / cmd cango
    RK.FL_enc_speed = RK.power_factor * float(wheel_front_left) * float(RK.encoder_resolution) / (2 * math.pi) # motor power / cmd cango
    RK.RR_enc_speed = RK.power_factor * float(wheel_rear_right) * float(RK.encoder_resolution) / (2 * math.pi) # motor power / cmd cango
    RK.RL_enc_speed = RK.power_factor * float(wheel_rear_left) * float(RK.encoder_resolution) / (2 * math.pi) # motor power / cmd cango

    #limit max power to 1000
    if abs(RK.FR_enc_speed) > 1000:
        RK.FR_enc_speed = 1000 * RK.FR_enc_speed / abs(RK.FR_enc_speed)
    if abs(RK.FL_enc_speed) > 1000:
        RK.FL_enc_speed = 1000 * RK.FL_enc_speed / abs(RK.FL_enc_speed)
    if abs(RK.RR_enc_speed) > 1000:
        RK.RR_enc_speed = 1000 * RK.RR_enc_speed / abs(RK.RR_enc_speed)
    if abs(RK.RL_enc_speed) > 1000:
        RK.RL_enc_speed = 1000 * RK.RL_enc_speed / abs(RK.RL_enc_speed)
    


    ## Additional data not needed for setting drive commands


    
def panther_driver():
    global RK

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

    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback, queue_size=1)

    can_interface = rospy.get_param('~can_interface', 'panther_can')
    RK.robot_width = rospy.get_param('~robot_width', 0.682)
    RK.robot_length = rospy.get_param('~robot_length', 0.44)
    RK.wheel_radius = rospy.get_param('~wheel_radius', 0.1015)
    RK.encoder_resolution = rospy.get_param('~encoder_resolution', 30*400) 
    RK.power_factor = rospy.get_param('~power_factor', 0.04166667)

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

    robot_x_pos = 0
    robot_y_pos = 0
    robot_th_pos = 0

    while not rospy.is_shutdown():
        try:
            rospy.logerr("RK.FL_enc_speed: %s" % RK.FL_enc_speed)
            rospy.logerr("RK.FR_enc_speed: %s" % RK.FR_enc_speed)
            rospy.logerr("RK.RL_enc_speed: %s" % RK.RL_enc_speed)
            rospy.logerr("RK.RR_enc_speed: %s" % RK.RR_enc_speed)

            front_controller.sdo['Cmd_CANGO'][2].raw = RK.FL_enc_speed
            front_controller.sdo['Cmd_CANGO'][1].raw = RK.FR_enc_speed
            rear_controller.sdo['Cmd_CANGO'][2].raw = RK.RL_enc_speed
            rear_controller.sdo['Cmd_CANGO'][1].raw = RK.RR_enc_speed

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

            wheel_FL_ang_pos = 2 * 3.14 * RK.wheel_radius * position_FL / RK.encoder_resolution 
            wheel_FR_ang_pos = 2 * 3.14 * RK.wheel_radius * position_FR / RK.encoder_resolution
            wheel_RL_ang_pos = 2 * 3.14 * RK.wheel_radius * position_RL / RK.encoder_resolution
            wheel_RR_ang_pos = 2 * 3.14 * RK.wheel_radius * position_RR / RK.encoder_resolution

            wheel_FL_ang_vel = 2 * 3.14 * RK.wheel_radius * speed_FL / RK.encoder_resolution
            wheel_FR_ang_vel = 2 * 3.14 * RK.wheel_radius * speed_FR / RK.encoder_resolution
            wheel_RL_ang_vel = 2 * 3.14 * RK.wheel_radius * speed_RL / RK.encoder_resolution
            wheel_RR_ang_vel = 2 * 3.14 * RK.wheel_radius * speed_RR / RK.encoder_resolution

            RK.wheels_angular_velocity = [wheel_FL_ang_vel,wheel_FR_ang_vel,wheel_RR_ang_vel,wheel_RL_ang_vel]

            linear_velocity_x_ = -(-wheel_FL_ang_vel + wheel_FR_ang_vel - wheel_RL_ang_vel + wheel_RR_ang_vel) * (RK.wheel_radius/4)
            linear_velocity_y_ = (wheel_FL_ang_vel + wheel_FR_ang_vel - wheel_RL_ang_vel - wheel_RR_ang_vel) * (RK.wheel_radius/4)
            angular_velocity_z_ = (wheel_FL_ang_vel + wheel_FR_ang_vel + wheel_RL_ang_vel +wheel_RR_ang_vel) * (RK.wheel_radius/(4 * (RK.robot_width + RK.robot_length)))

            delta_heading = angular_velocity_z_ / loop_rate # [radians]
            delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) / loop_rate # [m]
            delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) / loop_rate # [m]

            robot_x_pos = robot_x_pos + delta_x
            robot_y_pos = robot_y_pos + delta_y
            robot_th_pos = robot_th_pos + delta_heading

            qz, qw = euler_to_quaternion(0,0,robot_th_pos)

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