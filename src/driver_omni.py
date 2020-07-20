#!/usr/bin/env python
import math
import rospy
import canopen
# import numpy as np
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

class ForwardKinematics():
    def __init__(self):
        self.lin_x = 0
        self.lin_y = 0
        self.ang_z = 0
        self.robot_width = 0
        self.robot_length = 0
        self.wheel_radius = 0
        self.encoder_resolution = 0
        self.FR_enc_speed = 0 #front right encoder speed
        self.FL_enc_speed = 0
        self.RR_enc_speed = 0
        self.RL_enc_speed = 0
        self.robot_x_pos = 0
        self.robot_y_pos = 0
        self.robot_th_pos = 0

class InverseKinematics():
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
        self.roller_radius = 0
        #I am using left to right numbering starting from front left wheel ending at rear right
        self.roller_vel = np.zeros(4)
        self.wheels_angular_velocity = np.zeros(4)
        self.generalized_wheel_vel_E = np.zeros(4) #In wheel frame E (local frame) (wheel)
        self.generalized_wheel_vel_X = np.zeros(4) #In wheel frame X (global frame) (robot)
        self.omegas = np.zeros(4)
    
    def calcRollerVel(self):
        for i in range(4):
            self.roller_vel[i] = self.roller_radius * self.wheels_angular_velocity[i] / math.cos(0.785398) # 45deg
            self.generalized_wheel_vel_E[i] = self.wheel_radius * self.wheels_angular_velocity[i]
            self.generalized_wheel_vel_X[i] = self.roller_vel[i] * math.sin(0.785398) # angle beetwen roller and X coordinate vector (ahead of robot)

    


FK = ForwardKinematics()
IK = InverseKinematics()

def cmd_vel_callback(data):
    # forward kinematics
    global FK
    FK.lin_x = data.linear.x # m/s
    FK.lin_y = data.linear.y
    FK.ang_z = data.angular.z # rad/s

    wheel_front_right = (1/FK.wheel_radius) * (FK.lin_x + FK.lin_y + (FK.robot_width + FK.robot_length)*FK.ang_z)# rad/s
    wheel_front_left = (1/FK.wheel_radius) * (FK.lin_x - FK.lin_y - (FK.robot_width + FK.robot_length)*FK.ang_z)  
    wheel_rear_right = (1/FK.wheel_radius) * (FK.lin_x - FK.lin_y + (FK.robot_width + FK.robot_length)*FK.ang_z)
    wheel_rear_left = (1/FK.wheel_radius) * (FK.lin_x + FK.lin_y - (FK.robot_width + FK.robot_length)*FK.ang_z)
    

    FK.FR_enc_speed = float(wheel_front_right) * float(FK.encoder_resolution) / (2 * math.pi) # tics/s
    FK.FL_enc_speed = float(wheel_front_left) * float(FK.encoder_resolution) / (2 * math.pi) # tics/s
    FK.RR_enc_speed = float(wheel_rear_right) * float(FK.encoder_resolution) / (2 * math.pi) # tics/s
    FK.RL_enc_speed = float(wheel_rear_left) * float(FK.encoder_resolution) / (2 * math.pi) # tics/s

    ## Additional data not needed for setting drive commands


    
def panther_driver():
    global FK
    global IK

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
    FK.robot_width = IK.robot_width = rospy.get_param('~robot_width', 0.67)
    FK.robot_length = IK.robot_length = rospy.get_param('~robot_length', 0.44)
    FK.wheel_radius = IK.wheel_radius = rospy.get_param('~wheel_radius', 0.1825)
    FK.encoder_resolution = IK.encoder_resolution = rospy.get_param('~encoder_resolution', 38400)
    IK.roller_radius = rospy.get_param('~roller_radius', 0.1)

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
            front_controller.sdo['Cmd_CANGO'][1].raw = IK.FL_enc_speed
            front_controller.sdo['Cmd_CANGO'][2].raw = IK.FR_enc_speed
            rear_controller.sdo['Cmd_CANGO'][1].raw = IK.RL_enc_speed
            rear_controller.sdo['Cmd_CANGO'][2].raw = IK.RR_enc_speed

            battery_msg.voltage = float(front_controller.sdo[0x210D][2].raw)/10
            battery_msg.current = float(front_controller.sdo['Qry_BATAMPS'][1].raw)/10
            battery_publisher.publish(battery_msg)

            #inverse kinematics

            # position_FL = front_controller.sdo['Qry_ABCNTR'][1].raw
            # position_FR = front_controller.sdo['Qry_ABCNTR'][2].raw
            # position_RL = rear_controller.sdo['Qry_ABCNTR'][1].raw
            # position_RR = rear_controller.sdo['Qry_ABCNTR'][2].raw

            # speed_FL = front_controller.sdo['Qry_ABSPEED'][1].raw
            # speed_FR = front_controller.sdo['Qry_ABSPEED'][2].raw
            # speed_RL = rear_controller.sdo['Qry_ABSPEED'][1].raw
            # speed_RR = rear_controller.sdo['Qry_ABSPEED'][2].raw

            # motamps_FL = float(front_controller.sdo['Qry_MOTAMPS'][1].raw)/10
            # motamps_FR = float(front_controller.sdo['Qry_MOTAMPS'][2].raw)/10
            # motamps_RL = float(rear_controller.sdo['Qry_MOTAMPS'][1].raw)/10
            # motamps_RR = float(rear_controller.sdo['Qry_MOTAMPS'][2].raw)/10

            # joint_state_msg.position = [position_FL, position_FR, position_RL, position_RR]
            # joint_state_msg.velocity = [speed_FL, speed_FR, speed_RL, speed_RR]
            # joint_state_msg.effort = [motamps_FL, motamps_FR, motamps_RL, motamps_RR]
            # joint_state_publisher.publish(joint_state_msg)

            # wheel_FL_ang_pos = 2 * 3.14 * IK.wheel_radius * position_FL / IK.encoder_resolution 
            # wheel_FR_ang_pos = 2 * 3.14 * IK.wheel_radius * position_FR / IK.encoder_resolution
            # wheel_RL_ang_pos = 2 * 3.14 * IK.wheel_radius * position_RL / IK.encoder_resolution
            # wheel_RR_ang_pos = 2 * 3.14 * IK.wheel_radius * position_RR / IK.encoder_resolution

            # wheel_FL_ang_vel = 2 * 3.14 * IK.wheel_radius * speed_FL / IK.encoder_resolution
            # wheel_FR_ang_vel = 2 * 3.14 * IK.wheel_radius * speed_FR / IK.encoder_resolution
            # wheel_RL_ang_vel = 2 * 3.14 * IK.wheel_radius * speed_RL / IK.encoder_resolution
            # wheel_RR_ang_vel = 2 * 3.14 * IK.wheel_radius * speed_RR / IK.encoder_resolution

            # IK.wheels_angular_velocity = [wheel_FL_ang_vel,wheel_FR_ang_vel,wheel_RR_ang_vel,wheel_RL_ang_vel]

            # robot_angular_pos = (wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / robot_width
            # robot_angular_vel = (wheel_R_ang_vel - wheel_L_ang_vel) * wheel_radius / robot_width
            

            # T_prim = np.array([[1,0,-(IK.robot_length / 2)],[0,1,(IK.robot_width / 2)]])
            # T = np.dot(np.linalg.inv(wiTPi),np.linalg.inv(PiTR),T_prim)
            # T_plus = np.matmul(np.linalg.inv(np.matmul(np.transpose(T),T)),np.transpose(T))
            # linear_x_R, linear_y_R , angular_z_R = np.matmul(T_plus,np.transpose(omegas,vir))

            # linear_x = (FK.omegas[0] + FK.omegas[1] + FK.omegas[2] + FK.omegas[3]) * (IK.wheel_radius/4)
            # linear_y = ( -FK.omegas[0] + FK.omegas[1] + FK.omegas[2] - FK.omegas[3]) * (IK.wheel_radius/4)
            # angular_z = ( -FK.omegas[0] + FK.omegas[1] - FK.omegas[2] + FK.omegas[3]) * (IK.wheel_radius/(4 * (IK.robot_width + IK.robot_length)))

            # robot_vel = math.pow((math.pow(linear_x,2)+math.pow(linear_y,2)),0.5)
            # angle = math.atan2(linear_y,linear_x)
            # # convert robot angular pose to quaternion and publish to pose_msg

            # robot_x_pos = robot_x_pos + linear_x / loop_rate
            # robot_y_pos = robot_y_pos + linear_y / loop_rate
            # robot_th_pos = robot_th_pos + angular_z /loop_rate


            pose_msg.position.x = 0.0 #robot_x_pos
            pose_msg.position.y = 0.0 #robot_y_pos
            pose_publisher.publish(pose_msg)
        except:
            rospy.logerr("CAN protocol error")

        rate.sleep()

if __name__ == '__main__':
    try:
        panther_driver()
    except rospy.ROSInterruptException:
        pass