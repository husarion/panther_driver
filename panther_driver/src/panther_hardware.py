#!/usr/bin/python3

import time

import RPi.GPIO as GPIO


input_pin = 15
output_pin = 31

GPIO.setmode(GPIO.BOARD)
GPIO.setup(output_pin, GPIO.OUT)
GPIO.setup(input_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

while(True):
    try:
        GPIO.output(output_pin, GPIO.input(input_pin))
        time.sleep(0.01)
    except:
        GPIO.cleanup()
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(output_pin, GPIO.OUT)
        GPIO.setup(input_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.cleanup()