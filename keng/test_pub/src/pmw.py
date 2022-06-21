#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int64

class PWM():
    def __init__(self):
            self.vl_sub = rospy.Subscriber('/control_left_wheel/command',Float64,self.callback_l)
            self.vr_sub = rospy.Subscriber('/control_right_wheel/command',Float64,self.callback_r)
            self.pwm_wheel_left_pub = rospy.Publisher('/pwm_l',Float64)
            self.pwm_wheel_right_pub = rospy.Publisher('/pwm_r',Float64)
            self.p_l = Float64()
            self.p_r = Float64()
            self.vl = Float64()
            self.vr = Float64()
            self.rate = rospy.Rate(1)
        
    def callback_l(self,msg):
        self.vl.data = msg.data

    def callback_r(self,msg):
        self.vr.data = msg.data

    def pub_pwm(self):
        while not rospy.is_shutdown():
            self.convert_pwm()
            self.pwm_wheel_left_pub.publish(self.p_l)
            self.pwm_wheel_right_pub.publish(self.p_r)
            self.rate.sleep()

    def convert_pwm(self):
        PPR = 40120
        R = 0.12/2
        self.p_l.data = self.vl.data*255/0.083
        self.p_r.data = self.vr.data*255/0.083
        if self.p_l.data >= 255:
            self.p_l.data = 255
        if self.p_r.data >= 255:
            self.p_r.data = 255


if __name__ == '__main__':
    rospy.init_node('pwm_converter_node',anonymous=True)
    bv = PWM()
    bv.pub_pwm()
