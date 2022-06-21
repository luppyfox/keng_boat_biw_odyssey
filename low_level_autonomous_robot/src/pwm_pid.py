#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import sys
import signal
from std_msgs.msg import Int64
from std_msgs.msg import Float32

class PWM_PID():
   def __init__(self):
      self.encL_a = 12                             #pin
      self.encL_b = 16                             #pin
      self.encR_a = 20                             #pin
      self.encR_b = 21                             #pin
      self.AN2 = 13                                #pin PWM
      self.DIG2 = 26                               #pin DIR
      self.AN1 = 23                                #pin PWM
      self.DIG1 = 24                               #pin DIR

      self.total_pulse = 1967

      self.prevT_L = 0
      self.prevT_R = 0
      self.posPrev_L = 0
      self.vlFilt = 0.0
      self.vlPrev = 0.0
      self.vt_L = 0.0
      self.posPrev_R = 0
      self.vrFilt = 0.0
      self.vrPrev = 0.0
      self.vt_R = 0.0

      self.pulse_L = 0
      self.pulse_R = 0
      self.pulse_L_state = 0

      self.kp = 35.0                               #check this value again
      self.ki = 650.0                              #check this value again

      self.e_L = 0.0
      self.e_R = 0.0
      self.eintegral_L = 0.0
      self.eintegral_R = 0.0

      GPIO.setmode(GPIO.BCM)
      GPIO.setwarnings(False)
      GPIO.setup(self.AN2, GPIO.OUT)               # set pin as output
      GPIO.setup(self.DIG2, GPIO.OUT)              # set pin as output
      GPIO.setup(self.AN1, GPIO.OUT)
      GPIO.setup(self.DIG1, GPIO.OUT)
      GPIO.setup(self.encL_a, GPIO.IN)
      GPIO.setup(self.encR_a, GPIO.IN)
      GPIO.setup(self.encL_b, GPIO.IN)
      GPIO.setup(self.encR_b, GPIO.IN)
      p2 = GPIO.PWM(self.AN2, 20)                  # set pwm for M2
      p1 = GPIO.PWM(self.AN1, 20)                  # set pwm for M1

   def signal_handler(self, sig, frame):
      GPIO.cleanup()
      sys.exit(0)

   def encL_a_callback(self, channel):
      if GPIO.input(self.encL_b) == 0:
         if GPIO.input(self.encL_a) == 0:
            self.pulse_L-=1
         else:
            self.pulse_L+=1
      print("pulse_R:          ", self.pulse_R, "         pulse_L:          ", self.pulse_L)

   def encR_a_callback(self, channel):
      if GPIO.input(self.encR_b) == 0:
         if GPIO.input(self.encR_a) == 0:
            self.pulse_R-=1
         else:
            self.pulse_R+=1
      print("pulse_R:          ", self.pulse_R, "         pulse_L:          ", self.pulse_L)

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