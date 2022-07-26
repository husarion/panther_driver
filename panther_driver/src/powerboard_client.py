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
SHDN_INIT = 16
AUX_PW_EN = 18
CHRG_EN = 19
VDIG_OFF = 21
DRIVER_EN = 23
E_STOP_RESET = 27


class PantherHardware:
    def __init__(self) -> None:
        # Setup GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(VMOT_ON, GPIO.OUT, initial=0)
        GPIO.setup(CHRG_SENSE, GPIO.IN)
        GPIO.setup(AUX_PW_EN, GPIO.OUT, initial=0)
        GPIO.setup(CHRG_EN, GPIO.OUT, initial=1)
        GPIO.setup(VDIG_OFF, GPIO.OUT, initial=0)
        GPIO.setup(DRIVER_EN, GPIO.OUT, initial=0)
        GPIO.setup(E_STOP_RESET, GPIO.IN) # USED AS I/O

        self.watchdog_on = False
        self.watchdog_pwm = PWMOutputDevice(WATCHDOG)
        self.toggle_watchdog()

        self.soft_stop_thread = threading.Thread(name='soft stop proc', target=self.soft_stop)
        self.soft_stop_thread.start()

        # Setup ROS Node
        rospy.init_node('panther_hardware')
        self.aux_power_enable_service = rospy.Service('/panther_hardware/aux_power_enable', SetBool, self.handle_aux_power_enable)
        self.charger_enable_service = rospy.Service('/panther_hardware/charger_enable', SetBool, self.handle_charger_enable)
        self.disable_digital_service = rospy.Service('/panther_hardware/disable_digital_power', SetBool, self.handle_disable_digital_power)
        self.motors_enable_service = rospy.Service('/panther_hardware/motors_enable', SetBool, self.handle_motors_enable)
        self.reset_e_stop_service = rospy.Service('/panther_hardware/reset_e_stop', Trigger, self.handle_reset_e_stop)
        self.toggle_e_stop_service = rospy.Service('/panther_hardware/toggle_e_stop', Trigger, self.handle_toggle_e_stop)

        self.e_stop_state_pub = rospy.Publisher('/panther_hardware/e_stop', Bool, queue_size=1)
        self.timer_e_stop = rospy.Timer(rospy.Duration(0.1), self.publish_e_stop_state)

        self.charger_state_pub = rospy.Publisher('/panther_hardware/charger_sens', Bool, queue_size=1)
        self.timer_charger = rospy.Timer(rospy.Duration(0.5), self.publish_charger_state)

        rospy.spin()

    def publish_e_stop_state(self, event=None):
        msg = Bool()
        msg.data = GPIO.input(E_STOP_RESET)
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
        return self.handle_set_bool_srv(req.data, VDIG_OFF, "Disable digital power")

    def handle_motors_enable(self, req: SetBoolRequest):
        return self.handle_set_bool_srv(req.data, DRIVER_EN, "Motors driver enable")

    def handle_reset_e_stop(self, req: TriggerRequest):
        # Read value before reset
        e_stop_val = GPIO.input(E_STOP_RESET)
        print(f"E-STOP state = {e_stop_val}")

        # Switch e-stop pin to output
        GPIO.setup(E_STOP_RESET, GPIO.OUT)

        response = self.handle_trigger_srv(True, E_STOP_RESET, "E-STOP reset")
        time.sleep(0.1)

        # Switch back to input after writing
        GPIO.setup(E_STOP_RESET, GPIO.IN)

        return response

    def handle_toggle_e_stop(self, req: TriggerRequest):
        self.toggle_watchdog()
        return TriggerResponse(True, f"Watchdog on: {self.watchdog_on}")
        

    def handle_set_bool_srv(self, reqest, pin, name):
        success = self.handle_gpio_write(reqest, pin, name)
        msg = ""
        if success:
            msg = f"{name} successfull"
        else:
            msg = f"{name} failed"

        return SetBoolResponse(success, msg)

    def handle_trigger_srv(self, reqest, pin, name):
        success = self.handle_gpio_write(reqest, pin, name)
        msg = ""
        if success:
            msg = f"{name} successfull"
        else:
            msg = f"{name} failed"

        return TriggerResponse(success, msg)


    def handle_gpio_write(self, reqest, pin, name):
        print(f"Requested {name} = {reqest}")
        
        # Try to write to pin
        try:
            GPIO.output(pin, reqest)
        except:
            rospy.logwarn(f"Error writing to {name} pin")
            return False
        
        # Check that the pin value is correct
        if GPIO.input(pin) == reqest:
            return True
        else: 
            return False

        
    def toggle_watchdog(self):
        if self.watchdog_on:
            self.watchdog_pwm.off()
            self.watchdog_on = False
        else:
            freqency = 10
            step = 1/freqency/2
            self.watchdog_pwm.blink(on_time=step,off_time=step)
            self.watchdog_on = True

    def soft_stop(self):
        button = Button(SHDN_INIT, pull_up = False)
        button.wait_for_press()
        os.system('sudo shutdown now')


if __name__ == '__main__':
    try:
        PantherHardware()
    except rospy.ROSInterruptException:
        pass
