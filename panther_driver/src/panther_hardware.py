#!/usr/bin/python3

import time

import RPi.GPIO as GPIO
from gpiozero import PWMOutputDevice
import rospy

from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


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
        self._watchdog_on = False
        self._watchdog_pwm = PWMOutputDevice(WATCHDOG)

    def turn_on(self) -> None:
        if not self._watchdog_on:
            frequency = 50
            step = 1 / frequency / 2
            self._watchdog_pwm.blink(on_time=step, off_time=step)
            self._watchdog_on = True

    def turn_off(self) -> None:
        if self._watchdog_on:
            self._watchdog_pwm.off()
            self._watchdog_on = False


class PantherHardware:
    def __init__(self) -> None:
        self.setup_gpio()

        self._watchdog = Watchdog()
        self._watchdog.turn_on()

        rospy.init_node("panther_hardware")

        self.e_stop_state_pub = rospy.Publisher(
            "/panther_hardware/e_stop", Bool, queue_size=1
        )
        self.timer_e_stop = rospy.Timer(rospy.Duration(0.1), self.publish_e_stop_state)

        self.charger_state_pub = rospy.Publisher(
            "/panther_hardware/charger_sens", Bool, queue_size=1
        )
        self.timer_charger = rospy.Timer(
            rospy.Duration(0.5), self.publish_charger_state
        )

        self.aux_power_enable_service = rospy.Service(
            "/panther_hardware/aux_power_enable", SetBool, self.handle_aux_power_enable
        )
        self.charger_enable_service = rospy.Service(
            "/panther_hardware/charger_enable", SetBool, self.handle_charger_enable
        )
        self.disable_digital_service = rospy.Service(
            "/panther_hardware/disable_digital_power",
            SetBool,
            self.handle_disable_digital_power,
        )
        self.motors_enable_service = rospy.Service(
            "/panther_hardware/motors_enable", SetBool, self.handle_motors_enable
        )
        self.fan_enable_service = rospy.Service(
            "/panther_hardware/fan_enable", SetBool, self.handle_fan_enable
        )
        self.reset_e_stop_service = rospy.Service(
            "/panther_hardware/reset_e_stop", Trigger, self.handle_reset_e_stop
        )
        self.trigger_e_stop_service = rospy.Service(
            "/panther_hardware/trigger_e_stop", Trigger, self.handle_trigger_e_stop
        )

        self.motor_start_sequence()

        rospy.spin()

    def setup_gpio(self) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(VMOT_ON, GPIO.OUT, initial=0)
        GPIO.setup(CHRG_SENSE, GPIO.IN)
        GPIO.setup(FAN_SW, GPIO.OUT, initial=0)
        GPIO.setup(AUX_PW_EN, GPIO.OUT, initial=0)
        GPIO.setup(CHRG_EN, GPIO.OUT, initial=1)
        GPIO.setup(VDIG_OFF, GPIO.OUT, initial=0)
        GPIO.setup(DRIVER_EN, GPIO.OUT, initial=0)
        GPIO.setup(E_STOP_RESET, GPIO.IN)  # USED AS I/O

    def motor_start_sequence(self) -> None:
        """
        First start sequence for motors which is meant to power up modules in correct order
        """
        GPIO.output(VMOT_ON, 1)
        time.sleep(0.5)

        GPIO.output(DRIVER_EN, 1)
        time.sleep(0.2)

        GPIO.output(AUX_PW_EN, 1)

    def publish_e_stop_state(self, event=None) -> None:
        msg = Bool()
        msg.data = self.read_e_stop_pin()
        self.e_stop_state_pub.publish(msg)

    def publish_charger_state(self, event=None) -> None:
        msg = Bool()
        msg.data = GPIO.input(CHRG_SENSE)
        self.charger_state_pub.publish(msg)

    def handle_aux_power_enable(self, req: SetBoolRequest) -> SetBoolResponse:
        return self.handle_set_bool_srv(req, AUX_PW_EN, "Aux power enable")

    def handle_charger_enable(self, req: SetBoolRequest) -> SetBoolResponse:
        return self.handle_set_bool_srv(req, CHRG_EN, "Charger enable")

    def handle_disable_digital_power(self, req: SetBoolRequest) -> SetBoolResponse:
        return self.handle_set_bool_srv(req, VDIG_OFF, "Digital power disable")

    def handle_motors_enable(self, req: SetBoolRequest) -> SetBoolResponse:
        return self.handle_set_bool_srv(req, DRIVER_EN, "Motors driver enable")

    def handle_fan_enable(self, req: SetBoolRequest) -> SetBoolResponse:
        return self.handle_set_bool_srv(req, FAN_SW, "Fan enable")

    def handle_trigger_e_stop(self, req: TriggerRequest) -> TriggerResponse:
        self._watchdog.turn_off()
        return TriggerResponse(True, f"E-STOP triggered, watchdog turned off")

    def handle_reset_e_stop(self, req: TriggerRequest) -> TriggerResponse:
        # Read value before reset
        e_stop_initial_val = self.read_e_stop_pin()

        # Check if E-STOP reset is needed
        if e_stop_initial_val == False:
            msg = "E-STOP was not triggered, reset is not needed"
            response = TriggerResponse(False, msg)
            return response

        # Switch e-stop pin to output
        GPIO.setup(E_STOP_RESET, GPIO.OUT)

        # Perform reset
        success = self.write_and_validate_gpio(True, E_STOP_RESET, "E-STOP reset")
        self._watchdog.turn_on()
        time.sleep(0.1)

        # Switch back to input after writing
        GPIO.setup(E_STOP_RESET, GPIO.IN)

        # Check if E-STOP reset succeeded
        e_stop_val = self.read_e_stop_pin()

        # Send correct response
        if e_stop_initial_val == e_stop_val or not success:
            msg = (
                "E-STOP reset not successful, state unchanged, check for pressed E-STOP buttons or other sources"
            )
            response = TriggerResponse(False, msg)
            return response

        msg = "E-STOP reset successful"
        response = TriggerResponse(success, msg)
        return response

    def handle_set_bool_srv(self, request, pin, name) -> SetBoolResponse:
        success = self.write_and_validate_gpio(request.data, pin, name)
        msg = ""
        if success:
            msg = f"{name} write {request.data} successful"
        else:
            msg = f"{name} write {request.data} failed"

        return SetBoolResponse(success, msg)

    def write_and_validate_gpio(self, value: bool, pin: int, name: str) -> bool:
        rospy.logdebug(f"Requested {name} = {value}")

        # Try to write to pin
        try:
            GPIO.output(pin, value)
        except:
            rospy.logwarn(f"Error writing to {name} pin")
            return False

        # Check that the pin value is correct
        if GPIO.input(pin) == value:
            return True
        else:
            return False

    def read_e_stop_pin(self) -> bool:
        """
        Function designed to ensure that reverse logic of E-STOP reading is used
        """
        return not GPIO.input(E_STOP_RESET)


if __name__ == "__main__":
    try:
        PantherHardware()
    except rospy.ROSInterruptException:
        pass
