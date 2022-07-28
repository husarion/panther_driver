#!/usr/bin/python3

import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

import os
import time
import threading
# import multiprocessing

import RPi.GPIO as GPIO
from gpiozero import Button, PWMOutputDevice, OutputDevice
from signal import pause
from std_msgs.msg import Bool

# Define pin names
VMOT_ON = 6
CHRG_SENSE = 7
WATCHDOG = 14
FAN_SW = 15
# SHDN_INIT = 16 Shutdown Init managed by systemd service
AUX_PW_EN = 18
CHRG_EN = 19
VDIG_OFF = 21
DRIVER_EN = 23
E_STOP_RESET = 27

class Watchdog:
    def __init__(self) -> None:
        self.watchdog_on = False
        self.watchdog_pwm = PWMOutputDevice(WATCHDOG)

    def turn_on(self):
        if not self.watchdog_on:
            freqency = 50
            step = 1/freqency/2
            self.watchdog_pwm.blink(on_time=step,off_time=step)
            self.watchdog_on = True

    def turn_off(self):
        if self.watchdog_on:
            self.watchdog_pwm.off()
            self.watchdog_on = False


class PantherHardware:
    def __init__(self) -> None:
        # Setup GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(VMOT_ON, GPIO.OUT, initial=0)
        GPIO.setup(CHRG_SENSE, GPIO.IN)
        GPIO.setup(FAN_SW, GPIO.OUT, initial=0)
        GPIO.setup(AUX_PW_EN, GPIO.OUT, initial=0)
        GPIO.setup(CHRG_EN, GPIO.OUT, initial=1)
        GPIO.setup(VDIG_OFF, GPIO.OUT, initial=0)
        GPIO.setup(DRIVER_EN, GPIO.OUT, initial=0)
        GPIO.setup(E_STOP_RESET, GPIO.IN) # USED AS I/O

        self.watchdog = Watchdog()
        self.watchdog.turn_on()

        # Setup ROS Node
        rospy.init_node('panther_hardware')

        # ROS Services
        self.aux_power_enable_service = rospy.Service('/panther_hardware/aux_power_enable', SetBool, self.handle_aux_power_enable)
        self.charger_enable_service = rospy.Service('/panther_hardware/charger_enable', SetBool, self.handle_charger_enable)
        self.disable_digital_service = rospy.Service('/panther_hardware/disable_digital_power', SetBool, self.handle_disable_digital_power)
        self.motors_enable_service = rospy.Service('/panther_hardware/motors_enable', SetBool, self.handle_motors_enable)
        self.fan_enable_service = rospy.Service('/panther_hardware/fan_enable', SetBool, self.handle_fan_enable)
        self.reset_e_stop_service = rospy.Service('/panther_hardware/reset_e_stop', Trigger, self.handle_reset_e_stop)
        self.trigger_e_stop_service = rospy.Service('/panther_hardware/trigger_e_stop', Trigger, self.handle_trigger_e_stop)

        # ROS Publishers with timers
        self.e_stop_state_pub = rospy.Publisher('/panther_hardware/e_stop', Bool, queue_size=1)
        self.timer_e_stop = rospy.Timer(rospy.Duration(0.1), self.publish_e_stop_state)

        self.charger_state_pub = rospy.Publisher('/panther_hardware/charger_sens', Bool, queue_size=1)
        self.timer_charger = rospy.Timer(rospy.Duration(0.5), self.publish_charger_state)

        self.first_start_sequence()

        rospy.spin()

    def first_start_sequence(self):
        """
        First start sequence for motors which is meant to power up modules in correct order
        """
        GPIO.output(VMOT_ON, 1)
        time.sleep(0.5)

        GPIO.output(DRIVER_EN, 1)
        time.sleep(0.2)

        GPIO.output(AUX_PW_EN, 1)


    def publish_e_stop_state(self, event=None):
        msg = Bool()
        msg.data = self.read_e_stop_pin()
        self.e_stop_state_pub.publish(msg)

    def publish_charger_state(self, event=None):
        msg = Bool()
        msg.data = GPIO.input(CHRG_SENSE)
        self.charger_state_pub.publish(msg)

    def handle_aux_power_enable(self, req: SetBoolRequest):
        return self.handle_set_bool_srv(req.data, AUX_PW_EN, "Aux power enable")

    def handle_charger_enable(self, req: SetBoolRequest):
        return self.handle_set_bool_srv(req.data, CHRG_EN, "Charger enable")

    def handle_disable_digital_power(self, req: SetBoolRequest):
        return self.handle_set_bool_srv(req.data, VDIG_OFF, "Digital power disable")

    def handle_motors_enable(self, req: SetBoolRequest):
        return self.handle_set_bool_srv(req.data, DRIVER_EN, "Motors driver enable")

    def handle_fan_enable(self, req: SetBoolRequest):
        return self.handle_set_bool_srv(req.data, FAN_SW, "Fan enable")

    
    def handle_trigger_e_stop(self, req: TriggerRequest):
        self.watchdog.turn_off()
        return TriggerResponse(True, f"E-STOP triggered, watchdog turned off")

    def handle_reset_e_stop(self, req: TriggerRequest):
        # Read value before reset
        e_stop_initial_val = self.read_e_stop_pin()

        # Check if E-STOP reset is needed
        print(e_stop_initial_val)
        if e_stop_initial_val == False:
            msg = "E-STOP was not triggered, reset is not needed"
            response = TriggerResponse(False, msg)
            return response

        # Switch e-stop pin to output
        GPIO.setup(E_STOP_RESET, GPIO.OUT)

        #Perform reset
        success = self.handle_gpio_write(True, E_STOP_RESET, "E-STOP reset")
        self.watchdog.turn_on()
        time.sleep(0.1)

        # Switch back to input after writing
        GPIO.setup(E_STOP_RESET, GPIO.IN)

        # Check if E-STOP reset succeeded
        e_stop_val = self.read_e_stop_pin()
        print(f"E-STOP state = {e_stop_val}")

        # Send correct response
        if e_stop_initial_val == e_stop_val or not success:
            msg = "E-STOP reset not successful, initial value is the same as after reset"
            response = TriggerResponse(False, msg)
            return response

        msg = "E-STOP reset successful"
        response = TriggerResponse(success, msg)
        return response

    def handle_set_bool_srv(self, request, pin, name):
        success = self.handle_gpio_write(request, pin, name)
        msg = ""
        if success:
            msg = f"{name} write {request.data} successful"
        else:
            msg = f"{name} write {request.data} failed"

        return SetBoolResponse(success, msg)

    def handle_trigger_srv(self, request, pin, name):
        success = self.handle_gpio_write(request, pin, name)
        msg = ""
        if success:
            msg = f"{name} successfull"
        else:
            msg = f"{name} failed"

        return TriggerResponse(success, msg)

    def handle_gpio_write(self, request, pin, name):
        print(f"Requested {name} = {request}")
        
        # Try to write to pin
        try:
            GPIO.output(pin, request)
        except:
            rospy.logwarn(f"Error writing to {name} pin")
            return False
        
        # Check that the pin value is correct
        if GPIO.input(pin) == request:
            return True
        else: 
            return False

    def read_e_stop_pin(self):
        """
        Function designed to ensure that reverse logic of E-STOP reading is used
        """
        return not GPIO.input(E_STOP_RESET)


if __name__ == '__main__':
    try:
        PantherHardware()
    except rospy.ROSInterruptException:
        pass
