#!/usr/bin/python3

import time

import RPi.GPIO as GPIO
import rospy

from std_msgs.msg import Bool

STAGE2_INPUT = 22
MOTOR_ON = 6


class PantherHardware:
    def __init__(self, name) -> None:
        self._setup_gpio()

        rospy.init_node(name)

        self._motor_on_pub = rospy.Publisher('~motor_on', Bool, queue_size=1)

        rospy.Timer(rospy.Duration(0.01), self._motor_on_callback)

    @staticmethod
    def _setup_gpio() -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR_ON, GPIO.OUT)
        GPIO.setup(STAGE2_INPUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def _motor_on_callback(self, *args) -> None:
        try:
            GPIO.output(MOTOR_ON, GPIO.input(STAGE2_INPUT))
            self._motor_on_pub.publish(GPIO.input(STAGE2_INPUT))
            time.sleep(0.01)
        except:
            GPIO.cleanup()
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(MOTOR_ON, GPIO.OUT)
            GPIO.setup(STAGE2_INPUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def main():
    panther_hardware = PantherHardware('panther_hardware')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
