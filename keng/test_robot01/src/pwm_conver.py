#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int64
from std_msgs.msg import Float64

class Conver_pwm():
    def __init__(self):
        self.sub_r = rospy.Subscriber('/control_right_wheel/command',Float64,self.callback_r,queue_size=1)
        self.sub_l = rospy.Subscriber('/control_left_wheel/command',Float64,self.callback_l,queue_size=1)
        self.vr1 = Float64()
        self.vl1 = Float64()
        self.pwm_to_r = rospy.Publisher('/pwm_r',Int64,queue_size=1)
        self.pwm_to_l = rospy.Publisher('/pwm_l',Int64,queue_size=1)
        self.right_pwm = Int64()
        self.left_pwm = Int64()
        
        self.rate = rospy.Rate(1)

    def callback_r(self, msg):
        self.vr1.data = msg.data
        
    def callback_l(self,msg):
        self.vl1.data = msg.data

    def pwm (self):
        self.right_pwm.data = int(self.vr1.data*680) + 3
        
        self.left_pwm.data = int(self.vl1.data*680) + 7
        

    def pub_pwm_vels(self):
        while not rospy.is_shutdown():
            self.pwm()
            self.pwm_to_r.publish(self.right_pwm)
            self.pwm_to_l.publish(self.left_pwm)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pwm_conv', anonymous=True)
    cv = Conver_pwm()
    cv.pub_pwm_vels()


