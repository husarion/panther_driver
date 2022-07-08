#!/usr/bin/python3

import os
import time
import threading

import RPi.GPIO as GPIO

lock = threading.Lock()
run = True

def toggle():
    frequency = 25
    pwm_watchdog_pin = 40
    GPIO.setup(pwm_watchdog_pin, GPIO.OUT)
    state = True
    my_run = True
    while my_run:
        GPIO.output(pwm_watchdog_pin, state)
        state = not state
        time.sleep(1/frequency/2)
        with lock:
            my_run = run
    GPIO.cleanup(pwm_watchdog_pin)

if __name__ == '__main__':
    shutdown_pin = 36
    # os.system('sudo chown root.gpio /dev/gpiomem')
    # os.system('sudo chmod -R g+rw /dev/gpiomem')


    GPIO.setmode(GPIO.BOARD)
    # GPIO.setup(shutdown_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    motor_on_pin = 31

    GPIO.setup(motor_on_pin, GPIO.OUT)

    GPIO.output(motor_on_pin, GPIO.HIGH)
    GPIO.input(motor_on_pin)  # returns 1


    t = threading.Thread(name='child procs', target=toggle)
    t.start()

    # while True:
    #     try:
    #         if not GPIO.input(shutdown_pin):
    #             time.sleep(0.1)
    #             if not GPIO.input(shutdown_pin):
    #                 break
    #         time.sleep(0.5)
    #     except Exception as e:
    #         pass
    # with lock:
    #     run = False
    # time.sleep(1)
        
    # GPIO.cleanup(shutdown_pin)
    # os.system(f'sudo shutdown now')


