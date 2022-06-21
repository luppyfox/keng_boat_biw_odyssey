#! /usr/bin/env python3
import rospy
import time
import serial
from std_msgs.msg import Float32,Int64
from geometry_msgs.msg import Twist

class Pyserial():
    def __init__(self):
        self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)
        self.pub_enc_L = rospy.Publisher('/enc_L', Int64, queue_size=1)
        self.pub_enc_R = rospy.Publisher('/enc_R', Int64, queue_size=1)
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback) 
        self.cmd_vel = Twist()
        self.vl = Float32()
        self.vr = Float32()
        self.rpm = None
        self.pulse_L = Int64()
        self.pulse_R = Int64()
        self.rate = rospy.Rate(100)
        self.write_read()
    
    def callback(self,msg):
        self.cmd_vel = msg

    def send_rpm(self,rpm):
        self.arduino.write(bytes(rpm, 'utf-8'))
        time.sleep(0.1)
    
    def pub(self):
        #rospy.loginfo("vel: " + self.rpm)
        self.send_rpm(self.rpm)

    def convert_vel(self):
        self.vl = self.cmd_vel.linear.x - (self.cmd_vel.angular.z*(0.308/2))
        self.vr = self.cmd_vel.linear.x + (self.cmd_vel.angular.z*(0.308/2))
        self.rpm = str(self.vr*100) + "," + str(self.vl*100)

    def read_pulses(self):
        rawdata = str(self.arduino.readline())
        txt = rawdata.replace("b'","")
        txt = txt.replace("\\r\\n'","")

        if txt != "'":
            txt = txt.split(" ")
            if txt[1] == str(1):
                self.pulse_R.data = int(txt[0])
                self.pub_enc_R.publish(self.pulse_R)
            if txt[3] == str(1):
                self.pulse_L.data = int(txt[2])
                self.pub_enc_L.publish(self.pulse_L)
            #rospy.loginfo("pulse_L: " + str(self.pulse_L) + ", pulse_R: " + str(self.pulse_R))

    def write_read(self):
        while not rospy.is_shutdown():
            self.convert_vel()
            self.pub()
            self.read_pulses()
            self.rate.sleep()

        self.rpm = str(999) + "," + str(999)
        rospy.loginfo("reset all pulse")
        self.send_rpm(self.rpm)
        self.arduino.close()

if __name__ == "__main__":
    rospy.init_node('pyserial_node', anonymous=True)
    pyserial = Pyserial()

            



    

    
        

