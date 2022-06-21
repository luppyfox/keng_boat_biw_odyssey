#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import sys
import signal

encL_a = 12
encL_b = 16
encR_a = 20
encR_b = 21
AN2 = 13
DIG2 = 26
AN1 = 23
DIG1 = 24

pulse_L = 0
pulse_R = 0
pulse_L_state = 0
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(AN2, GPIO.OUT)               # set pin as output
GPIO.setup(DIG2, GPIO.OUT)              # set pin as output
GPIO.setup(AN1, GPIO.OUT)
GPIO.setup(DIG1, GPIO.OUT)
GPIO.setup(encL_a, GPIO.IN)
GPIO.setup(encR_a, GPIO.IN)
GPIO.setup(encL_b, GPIO.IN)
GPIO.setup(encR_b, GPIO.IN)
p2 = GPIO.PWM(AN2, 20)                  # set pwm for M2
p1 = GPIO.PWM(AN1, 20)
def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def encL_a_callback(channel):
    global pulse_L
    if GPIO.input(encL_b) == 0:
        if GPIO.input(encL_a) == 0:
            pulse_L-=1
        else:
            pulse_L+=1
    print("pulse_R:          ", pulse_R, "         pulse_L:          ", pulse_L)

def encR_a_callback(channel):
    global pulse_R
    if GPIO.input(encR_b) == 0:
        if GPIO.input(encR_a) == 0:
            pulse_R-=1
        else:
            pulse_R+=1
    print("pulse_R:          ", pulse_R, "         pulse_L:          ", pulse_L)

if __name__ == '__main__':
    while 1:
        GPIO.output(DIG1, GPIO.LOW)
        p1.start(100)
        GPIO.output(DIG2, GPIO.LOW)
        p2.start(100)
        GPIO.add_event_detect(encR_a, GPIO.BOTH, 
                callback=encR_a_callback, bouncetime=1)
        GPIO.add_event_detect(encL_a, GPIO.BOTH, 
                callback=encL_a_callback, bouncetime=1)
        signal.signal(signal.SIGINT, signal_handler)
        signal.pause()