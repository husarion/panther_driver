#!/usr/bin/python3

import canopen
import math
from numpy import NaN
import yaml

import rospy
import tf2_ros

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import JointState

from ClassicKinematics import PantherClassic
from MecanumKinematics import PantherMecanum
from MixedKinematics import PantherMix


def sign(x):
    x_sign = bool(x > 0) - bool(x < 0)
    return x_sign if x_sign != 0 else 1

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


def read_file(path):
    with open(path, 'r') as file:
        data = file.read().rstrip()

    file.close()

    return int(data)

def get_ADC_measurement(name: str, config_file):
    data = config_file[name]
    path = data["path"]
    raw_value = read_file(path)
    value = raw_value * data["LSB"]

    return value

def publish_battery_msg(bat_pub, present, V_bat=NaN, temp_bat=NaN, Ibat=NaN):
    battery_msg = BatteryState()
    if present:
        battery_msg.header.stamp=rospy.Time.now()
        battery_msg.voltage=V_bat
        battery_msg.temperature=temp_bat
        battery_msg.current=Ibat
        battery_msg.percentage=(battery_msg.voltage-32)/10
        battery_msg.capacity=20
        battery_msg.design_capacity=20
        battery_msg.charge=battery_msg.percentage*battery_msg.design_capacity
        battery_msg.power_supply_status
        battery_msg.power_supply_health
        battery_msg.power_supply_technology=3
        battery_msg.present=True
    else:
        battery_msg.header.stamp=rospy.Time.now()
        battery_msg.voltage=NaN
        battery_msg.temperature=NaN
        battery_msg.current=NaN
        battery_msg.percentage=NaN
        battery_msg.capacity=NaN
        battery_msg.design_capacity=NaN
        battery_msg.charge=NaN
        battery_msg.power_supply_status
        battery_msg.power_supply_health
        battery_msg.power_supply_technology=3
        battery_msg.present=False

    bat_pub.publish(battery_msg)
    
def voltage_to_deg(V_temp):
    # Source: https://electronics.stackexchange.com/questions/323043/how-to-calculate-temperature-through-ntc-thermistor-without-its-datasheet
    A = 298.15
    B = 3950
    U_supply = 3.28
    R1 = 10000
    R0 = 10000

    if  V_temp == 0 or V_temp >= U_supply:
        print("Temperature measurement error")
        return NaN

    R_therm = (V_temp * R1) / (U_supply - V_temp)

    # rospy.loginfo(f"U_meas={V_temp}, R_therm={R_therm}")
    return (A*B / (A*math.log(R_therm/R0)+B)) - 273.15

def driverNode():

    rospy.init_node('~', anonymous=False)
    kinematics_type = rospy.get_param('~wheel_type', "classic")
    odom_frame = rospy.get_param('~odom_frame', "odom")
    base_link_frame = rospy.get_param('~base_link_frame', "base_link")
    motor_torque_constant = rospy.get_param('~motor_torque_constant', 2.6149)
    publish_tf = rospy.get_param('~publish_tf', True)
    publish_odometry = rospy.get_param('~publish_odometry', True)
    publish_pose = rospy.get_param('~publish_pose', True)

    RK = factory(kinematics_type)
    br = tf2_ros.TransformBroadcaster()
    tf = TransformStamped()

    # Battery
    battery_publisher = rospy.Publisher('battery', BatteryState, queue_size=1)
    battery1_publisher = rospy.Publisher('battery1', BatteryState, queue_size=1)
    battery2_publisher = rospy.Publisher('battery2', BatteryState, queue_size=1)    

    if rospy.has_param('~measurements_file'):
        measurements_file = rospy.get_param('~measurements_file')
    else:
        rospy.logerr("measurements_file not defined, can not start collecting ADC measurements")
        return

    with open(measurements_file, "r") as stream:
        try:
            config_file = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    # --

    joint_state_publisher = rospy.Publisher(
        'joint_states', JointState, queue_size=1)
    joint_state_msg = JointState()
    joint_state_msg.header.frame_id = base_link_frame
    joint_state_msg.name = [
        'front_left_wheel_joint',
        'front_right_wheel_joint',
        'rear_left_wheel_joint',
        'rear_right_wheel_joint'
    ]

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
    RK.encoder_resolution = rospy.get_param('~encoder_resolution', 400*4)
    RK.gear_ratio = rospy.get_param('~gear_ratio', 30.08)
    RK.power_factor = rospy.get_param('~power_factor', 0.04166667)

    if rospy.has_param('~eds_file'):
        eds_file = rospy.get_param('~eds_file')
    else:
        rospy.logerr("eds_file not defined, can not start CAN interface")
        return

    loop_rate = 20
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

    wheel_pos = [0.0, 0.0, 0.0, 0.0]
    wheel_vel = [0.0, 0.0, 0.0, 0.0]
    wheel_curr = [0.0, 0.0, 0.0, 0.0]

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

            # Get battery Data
            try:
                Idriv1 = float(front_controller.sdo['Qry_BATAMPS'][1].raw)/10
                Idriv2 = float(rear_controller.sdo['Qry_BATAMPS'][1].raw)/10
            except:
                rospy.logwarn("Error getting battery data from CAN")

            try:
                V_bat1 = get_ADC_measurement("BAT1_voltage", config_file)
                V_bat2 = get_ADC_measurement("BAT2_voltage", config_file)
                V_temp_bat1 = get_ADC_measurement("BAT1_temp", config_file)
                V_temp_bat2 = get_ADC_measurement("BAT2_temp", config_file)
                Icharge_bat1 = get_ADC_measurement("BAT1_charge_current", config_file)
                Icharge_bat2 = get_ADC_measurement("BAT2_charge_current", config_file)
                Idig = get_ADC_measurement("IDIG_current", config_file)
            except:
                rospy.logerr("Battery ADC measurement error excep")

            # Try Calculate and publish BAT data
            try: 
                # Check battery num
                if V_temp_bat2 > 3.2: # ONE Battery
                    rospy.loginfo("One bat detected")

                    # Calculate Temp in deg of Celcius
                    temp_bat1 = voltage_to_deg(V_temp_bat1)

                    Ibat1 =  -1 * ( Idriv1 + Idriv2 + Idig - Icharge_bat1)

                    rospy.loginfo(f"BATTERY LOG: Idig={Idig}, " +
                        f"Ibat1={Ibat1}, Idriv1={Idriv1}, Icharge_bat1={Icharge_bat1}, " +
                        f"V_bat1={V_bat1}, V_temp_bat1={V_temp_bat1}, temp_bat1={temp_bat1}, Ibat1={Ibat1}")

                    publish_battery_msg(battery1_publisher, True, V_bat1, temp_bat1, Ibat1)
                    publish_battery_msg(battery2_publisher, False)
                else:
                    rospy.loginfo("Two bat detected")

                    # Calculate Temp in deg of Celcius
                    temp_bat1 = voltage_to_deg(V_temp_bat1)
                    temp_bat2 = voltage_to_deg(V_temp_bat2)

                    V_diff = V_bat1 - V_bat2

                    if abs(V_diff) <= 0.2:
                        k = 0.5
                    elif V_diff > 0.2:
                        k = 1
                    elif V_diff < -0.2:
                        k = 0
                    else:
                        rospy.logger("V_difff out of range")

                    Ibat1 = -1 * ( Idriv1 + (k * Idig) - Icharge_bat1)
                    Ibat2 = -1 * ( Idriv2 + ((1-k) * Idig) - Icharge_bat2 )

                    rospy.loginfo(f"BATTERY LOG: k={k}, Idig={Idig}, " +
                        f"Ibat1={Ibat1}, Idriv1={Idriv1}, Icharge_bat1={Icharge_bat1}, " +
                        f"Ibat2={Ibat2}, Idriv2={Idriv2}, Icharge_bat2={Icharge_bat2}, " +
                        f"V_bat1={V_bat1}, V_temp_bat1={V_temp_bat1}, temp_bat1={temp_bat1}, Ibat1={Ibat1}, " +
                        f"V_bat2={V_bat2}, V_temp_bat2={V_temp_bat2}, temp_bat2={temp_bat2}, Ibat2={Ibat2}")

                    publish_battery_msg(battery1_publisher, True, V_bat1, temp_bat1, Ibat1)
                    publish_battery_msg(battery2_publisher, True, V_bat2, temp_bat2, Ibat2)

                    V_bat_avereage = (V_bat1+V_bat2)/2
                    temp_average = (temp_bat1+temp_bat2)/2
                    I_bat_average = (Ibat1+Ibat2)/2

                    publish_battery_msg(battery_publisher, True, V_bat_avereage, temp_average, I_bat_average)
            except:
                rospy.logerr("Error Calculating and publishing bat data")


            # query position
            try:
                wheel_pos[0] = front_controller.sdo['Qry_ABCNTR'][2].raw
            except:
                rospy.logwarn("Error reading front left controller sdo")
            try:
                wheel_pos[1] = front_controller.sdo['Qry_ABCNTR'][1].raw
            except:
                rospy.logwarn("Error reading front right controller sdo")
            try:
                wheel_pos[2] = rear_controller.sdo['Qry_ABCNTR'][2].raw
            except:
                rospy.logwarn("Error reading rear left controller sdo")
            try:  
                wheel_pos[3] = rear_controller.sdo['Qry_ABCNTR'][1].raw
            except:
                rospy.logwarn("Error reading rear right controller sdo")

            # query velocity
            try:
                wheel_vel[0] = front_controller.sdo['Qry_ABSPEED'][2].raw
            except:
                rospy.logwarn("Error reading front left controller sdo")
            try:
                wheel_vel[1] = front_controller.sdo['Qry_ABSPEED'][1].raw
            except:
                rospy.logwarn("Error reading front right controller sdo")
            try:
                wheel_vel[2] = rear_controller.sdo['Qry_ABSPEED'][2].raw
            except:
                rospy.logwarn("Error reading rear left controller sdo")
            try:  
                wheel_vel[3] = rear_controller.sdo['Qry_ABSPEED'][1].raw
            except:
                rospy.logwarn("Error reading rear right controller sdo")


            # query current
            # division by 10 is needed according to documentation
            try:
                wheel_curr[0] = front_controller.sdo['Qry_MOTAMPS'][2].raw / 10.0
            except:
                rospy.logwarn("Error reading current front left controller sdo")
            try:
                wheel_curr[1] = front_controller.sdo['Qry_MOTAMPS'][1].raw / 10.0
            except:
                rospy.logwarn("Error reading current front right controller sdo")
            try:
                wheel_curr[2] = rear_controller.sdo['Qry_MOTAMPS'][2].raw / 10.0
            except:
                rospy.logwarn("Error reading current rear left controller sdo")
            try:  
                wheel_curr[3] = rear_controller.sdo['Qry_MOTAMPS'][1].raw / 10.0
            except:
                rospy.logwarn("Error reading current rear right controller sdo")


            joint_state_msg.header.stamp = rospy.Time.now()

            # convert tics to rad
            joint_state_msg.position = [(2.0 * math.pi) * (pos / (RK.encoder_resolution * RK.gear_ratio)) for pos in wheel_pos]
            # convert RPM to rad/s
            joint_state_msg.velocity = [(2.0 * math.pi / 60) * (vel / RK.gear_ratio) for vel in wheel_vel]
            # convert to A to Nm
            joint_state_msg.effort = [
               wheel_curr[i] * motor_torque_constant * sign(wheel_vel[i]) for i in range(wheel_curr)]

            joint_state_publisher.publish(joint_state_msg)

            try:
                robot_x_pos, robot_y_pos, robot_th_pos = RK.inverseKinematics(
                    *joint_state_msg.velocity, dt_)
            except:
                rospy.logwarn("Couldn't get robot pose")
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

        except Exception as e:
            rospy.logerr(f"[Panther Driver] Error: {e}")
            err_count+=1
            if err_count >= 10:
                return

        rate.sleep()


if __name__ == '__main__':
    try:
        driverNode()
    except rospy.ROSInterruptException:
        pass
