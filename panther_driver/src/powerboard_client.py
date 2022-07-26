#!/usr/bin/python3

import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

import os
import time
import threading

import RPi.GPIO as GPIO
from gpiozero import Button, PWMOutputDevice, OutputDevice
from signal import pause

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
        GPIO.setup(VMOT_ON, GPIO.OUT)
        GPIO.setup(CHRG_SENSE, GPIO.IN)
        GPIO.setup(AUX_PW_EN, GPIO.OUT)
        GPIO.setup(CHRG_EN, GPIO.OUT)
        GPIO.setup(VDIG_OFF, GPIO.OUT)
        GPIO.setup(DRIVER_EN, GPIO.OUT)
        GPIO.setup(E_STOP_RESET, GPIO.IN) # USED AS I/O

        self.watchdog_pwm_thread = threading.Thread(name='watchdog_pwm proc', target=self.watchdog_pwm)
        self.watchdog_pwm_thread.start()

        self.soft_stop_thread = threading.Thread(name='soft stop proc', target=self.soft_stop)
        self.soft_stop_thread.start()

        # Setup ROS Node
        rospy.init_node('panther_hardware')
        self.motors_enable_service = rospy.Service('/panther_hardware/motors_enable', SetBool, self.handle_motors_enable)

        rospy.spin()


    def handle_motors_enable(self, req: SetBoolRequest):
        print(f"Requested DRIVER_EN = {req.data}")

        success = False
        
        # Try to write to DRIVER_EN pin
        try:
            GPIO.output(DRIVER_EN, GPIO.HIGH)
        except:
            message = "Error writing to DRIVER_EN pin"
            rospy.logwarn(message)
            return SetBoolResponse(False, message)
        
        # Check that the pin value is correct
        if GPIO.input(DRIVER_EN) == req.data:
            return SetBoolResponse(True, "Motors enable successful")
        else: 
            return SetBoolResponse(False, "Motors enable failed, try again")

        
    def watchdog_pwm(self):
        watchdog_pwm = PWMOutputDevice(WATCHDOG)
        freqency = 10
        step = 1/freqency/2
        watchdog_pwm.blink(on_time=step,off_time=step)

    def soft_stop(self):
        button = Button(SHDN_INIT, pull_up = False)
        button.wait_for_press()
        os.system('sudo shutdown now')


if __name__ == '__main__':
    try:
        PantherHardware()
    except rospy.ROSInterruptException:
        pass
