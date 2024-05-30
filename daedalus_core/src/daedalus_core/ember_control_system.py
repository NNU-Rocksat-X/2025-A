#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import rospy

GPIO.setmode(GPIO.BMC)
output_pin = 16 # Physical Pin no 36
GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

# 
def trigger_action():
    GPIO.output(output_pin, GPIO.HIGH)  
    time.sleep(0.1) 
    GPIO.output(output_pin, GPIO.LOW)  

# Call trigger_action when needed
trigger_action()

GPIO.cleanup()
