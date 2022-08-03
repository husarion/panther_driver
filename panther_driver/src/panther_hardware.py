#!/usr/bin/python3

import time

import RPi.GPIO as GPIO
from gpiozero import PWMOutputDevice
import rospy

from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from pinout import *

class Watchdog:
    def __init__(self) -> None:
        self._watchdog_on = False
        self._watchdog_pwm = PWMOutputDevice(WATCHDOG)

    def turn_on(self) -> None:
        if not self._watchdog_on:
            frequency = 50
            step = (1 / frequency) / 2
            self._watchdog_pwm.blink(on_time=step, off_time=step)
            self._watchdog_on = True

    def turn_off(self) -> None:
        if self._watchdog_on:
            self._watchdog_pwm.off()
            self._watchdog_on = False


class PantherHardware:
    def __init__(self) -> None:
        self._setup_gpio()
        self._motor_start_sequence()

        self._watchdog = Watchdog()
        self._watchdog.turn_on()

        rospy.init_node("panther_hardware")

        self._e_stop_state_pub = rospy.Publisher(
            "/panther_hardware/e_stop", Bool, queue_size=1
        )
        self._timer_e_stop = rospy.Timer(
            rospy.Duration(0.1), self._publish_e_stop_state
        )

        self._charger_state_pub = rospy.Publisher(
            "/panther_hardware/charger_sens", Bool, queue_size=1
        )
        self._timer_charger = rospy.Timer(
            rospy.Duration(0.5), self._publish_charger_state
        )

        self._aux_power_enable_srv = rospy.Service(
            "/panther_hardware/aux_power_enable", SetBool, self._aux_power_enable_cb
        )
        self._charger_enable_srv = rospy.Service(
            "/panther_hardware/charger_enable", SetBool, self._charger_enable_cb
        )
        self._digital_power_enable_srv = rospy.Service(
            "/panther_hardware/digital_power_enable",
            SetBool,
            self._digital_power_enable_cb,
        )
        self._motors_enable_srv = rospy.Service(
            "/panther_hardware/motors_enable", SetBool, self._motors_enable_cb
        )
        self._fan_enable_srv = rospy.Service(
            "/panther_hardware/fan_enable", SetBool, self._fan_enable_cb
        )
        self._e_stop_reset_srv = rospy.Service(
            "/panther_hardware/e_stop_reset", Trigger, self._e_stop_reset_cb
        )
        self._e_stop_trigger_srv = rospy.Service(
            "/panther_hardware/e_stop_trigger", Trigger, self._e_stop_trigger_cb
        )

        rospy.spin()

    def _setup_gpio(self) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(VMOT_ON, GPIO.OUT, initial=0)
        GPIO.setup(CHRG_SENSE, GPIO.IN)
        GPIO.setup(FAN_SW, GPIO.OUT, initial=0)
        GPIO.setup(AUX_PW_EN, GPIO.OUT, initial=0)
        GPIO.setup(CHRG_EN, GPIO.OUT, initial=1)
        GPIO.setup(VDIG_OFF, GPIO.OUT, initial=0)
        GPIO.setup(DRIVER_EN, GPIO.OUT, initial=0)
        GPIO.setup(E_STOP_RESET, GPIO.IN)  # USED AS I/O

    def _motor_start_sequence(self) -> None:
        """
        First start sequence for motors which is meant to power up modules in correct order
        """
        self._write_to_pin(VMOT_ON, 1)
        time.sleep(0.5)

        self._write_to_pin(DRIVER_EN, 1)
        time.sleep(0.2)

        self._write_to_pin(AUX_PW_EN, 1)

    def _publish_e_stop_state(self, event=None) -> None:
        msg = Bool()
        msg.data = self._read_pin(E_STOP_RESET)
        self._e_stop_state_pub.publish(msg)

    def _publish_charger_state(self, event=None) -> None:
        msg = Bool()
        msg.data = self._read_pin(CHRG_SENSE)
        self._charger_state_pub.publish(msg)

    def _aux_power_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        return self._handle_set_bool_srv(req.data, AUX_PW_EN, "Aux power enable")

    def _charger_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        return self._handle_set_bool_srv(req.data, CHRG_EN, "Charger enable")

    def _digital_power_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        return self._handle_set_bool_srv(req.data, VDIG_OFF, "Digital power enable")

    def _motors_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        return self._handle_set_bool_srv(req.data, DRIVER_EN, "Motors driver enable")

    def _fan_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        return self._handle_set_bool_srv(req.data, FAN_SW, "Fan enable")

    def _e_stop_trigger_cb(self, req: TriggerRequest) -> TriggerResponse:
        self._watchdog.turn_off()
        return TriggerResponse(True, f"E-STOP triggered, watchdog turned off")

    def _e_stop_reset_cb(self, req: TriggerRequest) -> TriggerResponse:
        if self._validate_gpio_pin(E_STOP_RESET, False):
            return TriggerResponse(
                True, "E-STOP was not triggered, reset is not needed"
            )

        self._reset_e_stop()

        if self._validate_gpio_pin(E_STOP_RESET, True):
            return TriggerResponse(
                False,
                "E-STOP reset not successful, state unchanged, check for pressed E-STOP buttons or other sources"
            )

        return TriggerResponse(True, "E-STOP reset successful")

    def _reset_e_stop(self) -> None:
        GPIO.setup(E_STOP_RESET, GPIO.OUT)
        self._watchdog.turn_on()

        # Sending False because of inverse logic
        self._write_to_pin(E_STOP_RESET, False)
        time.sleep(0.1)

        GPIO.setup(E_STOP_RESET, GPIO.IN)

    def _handle_set_bool_srv(self, value: bool, pin: int, name: str) -> SetBoolResponse:
        rospy.logdebug(f"Requested {name} = {value}")
        self._write_to_pin(pin, value)
        success = self._validate_gpio_pin(pin, value)

        msg = f"{name} write {value} failed"
        if success:
            msg = f"{name} write {value} successful"

        return SetBoolResponse(success, msg)

    def _validate_gpio_pin(self, pin: int, value: bool) -> bool:
        return self._read_pin(pin) == value

    def _read_pin(self, pin: int) -> bool:
        """
        Wrapper for GPIO.input() designed to ensure that reverse logic of some pins is used
        """
        if pin in inverse_logic_pins:
            return not GPIO.input(pin)
        return GPIO.input(pin)

    def _write_to_pin(self, pin: int, value: bool) -> None:
        """
        Wrapper for GPIO.output() designed to ensure that reverse logic of some pins is used
        """
        if pin in inverse_logic_pins:
            GPIO.output(pin, not value)
            return 
        GPIO.output(pin, value)


if __name__ == "__main__":
    try:
        PantherHardware()
    except rospy.ROSInterruptException:
        pass
