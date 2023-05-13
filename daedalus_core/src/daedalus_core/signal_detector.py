#!/usr/bin/env python

import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BCM)
class Detector:
    def __init__(self, pin_number, persistence_threshold):
        GPIO.setup(pin_number, GPIO.IN)
        self.pin_number = pin_number
        self.persistence_threshold = persistence_threshold

        self.persistence = 0
        self.state = False

    def detect(self):
       
            current_state = GPIO.input(self.pin_number)
            if (current_state):
                self.persistence += 1
            else:
                self.persistence = 0

            if self.persistence > self.persistence_threshold:
                self.state = True
            else:
                self.state = False

            

    def get_state(self):
    
        return self.state



def cleanup():
    GPIO.cleanup()


if __name__ == "__main__":
    p1 = Detector(5,20)
    p1.detect()

    while True:
        time.sleep(0.1)
        p1.detect()

        print (p1.get_state())
    
    cleanup()