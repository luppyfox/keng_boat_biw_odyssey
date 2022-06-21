#! /usr/bin/env python3 
from curses import KEY_PPAGE
import rospy 
from std_msgs.msg import Float32, Int64, Int32
from geometry_msgs.msg import Twist

class PID():
    def __init__(self):
        self.kp = 10
        self.ki = 50
        self.total_Pulse = 40120
        self.rate = rospy.Rate(20)
        self.L = 0.308
        self.R = 0.06

        self.prevT = rospy.get_time()
        self.currT = rospy.get_time()
        self.deltaT = self.currT - self.prevT 

        rospy.Subscriber("/enc_L", Int64, self.enc_L_callback)
        rospy.Subscriber("/enc_R", Int64, self.enc_R_callback) 
        self.pulse_L = Int64()
        self.pulse_R = Int64() 
       
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback) 
        self.cmd_vel = Twist()

        self.right_wheel_pub = rospy.Publisher('/pwm_R', Int32, queue_size=1)
        self.left_wheel_pub = rospy.Publisher('/pwm_L', Int32, queue_size=1)
        self.pwm_L = Int32()
        self.pwm_R = Int32()

        self.right_vt_pub = rospy.Publisher('/vt_R', Float32, queue_size=1)
        self.left_vt_pub = rospy.Publisher('/vt_L', Float32, queue_size=1)
        self.vt_L = Float32()
        self.vt_R = Float32()

        self.right_u_pub = rospy.Publisher('/rpm_R', Float32, queue_size=1)
        self.left_u_pub = rospy.Publisher('/rpm_L', Float32, queue_size=1)
        self.rpm_L = Float32()
        self.rpm_R = Float32()

        self.posPrev_L, self.posPrev_R = 0, 0
        self.filter_L, self.filter_R = 0, 0
        self.prev_L, self.prev_R = 0, 0
        self.eintegral_L, self.eintegral_R = 0, 0

        self.rpmL, self.rpmR = 0, 0
        self.v_L, self.v_R = 0, 0
    
    def enc_L_callback(self,msg):
        self.pulse_L = msg
    def enc_R_callback(self,msg):
        self.pulse_R = msg

    def callback(self,msg):
        self.cmd_vel = msg
    
    def spin(self):
        while not rospy.is_shutdown():

            # Get times
            self.currT = rospy.get_time()
            self.deltaT = self.currT - self.prevT 

            # Get counts/s
            counts_L = (self.pulse_L.data - self.posPrev_L)/self.deltaT 
            self.posPrev_L = self.pulse_L.data
            counts_R = (self.pulse_R.data - self.posPrev_R)/self.deltaT
            self.posPrev_R = self.pulse_R.data

            # Update times
            self.prevT = rospy.get_time()       

            # Convert count/s to rpm
            self.rpm_L.data = counts_L/self.total_Pulse*60
            self.rpm_R.data = counts_R/self.total_Pulse*60

            # Low-pass filter (25 Hz cutoff)
            self.filter_L = 0.854*self.filter_L + 0.0728*self.rpm_L.data + 0.0728*self.prev_L
            self.prev_L = self.rpm_L.data
            self.filter_R = 0.854*self.filter_R + 0.0728*self.rpm_R.data + 0.0728*self.prev_R
            self.prev_R = self.rpm_R.data 

            # Compute vt
            self.v_L = self.cmd_vel.linear.x - (self.cmd_vel.angular.z*(self.L/2))
            self.v_R = self.cmd_vel.linear.x + (self.cmd_vel.angular.z*(self.L/2))
            self.vt_L.data = (self.v_L*60)/(2*3.14*self.R)
            self.vt_R.data = (self.v_R*60)/(2*3.14*self.R)

            if self.vt_L.data >= 8:
                self.vt_L.data = 8

            if self.vt_R.data >= 8:
                self.vt_R.data = 8

            if self.vt_L.data <= -8:
                self.vt_L.data = -8

            if self.vt_R.data <= -8:
                self.vt_R.data = -8

            # Compute the control signal  
            error_L = self.vt_L.data - self.filter_L
            self.eintegral_L = self.eintegral_L + error_L*self.deltaT
            error_R = self.vt_R.data - self.filter_R
            self.eintegral_R = self.eintegral_R + error_R*self.deltaT    

            control_L = (self.kp*error_L) + (self.ki*self.eintegral_L)
            control_R = (self.kp*error_R) + (self.ki*self.eintegral_R) 

            # Set max-min and direction of PWM
            self.pwm_L = int(control_L)
            if self.pwm_L > 255:
                self.pwm_L = 255
            if self.pwm_L < -255:
                self.pwm_L = -255
            if self.vt_L.data == 0:
                self.pwm_L = 0
            
            self.pwm_R = int(control_R)
            if self.pwm_R > 255:
                self.pwm_R = 255
            if self.pwm_R < -255:
                self.pwm_R = -255
            if self.vt_R.data == 0:
                self.pwm_R = 0
            
            #Publish
            self.left_wheel_pub.publish(self.pwm_L)
            self.right_wheel_pub.publish(self.pwm_R)
            self.left_vt_pub.publish(self.vt_L)
            self.right_vt_pub.publish(self.vt_R)
            self.left_u_pub.publish(self.rpm_L)
            self.right_u_pub.publish(self.rpm_R)          
            self.rate.sleep()                                   


if __name__ == '__main__': 
    rospy.init_node('test_pid' ,anonymous=True)
    pid = PID()
    pid.spin()