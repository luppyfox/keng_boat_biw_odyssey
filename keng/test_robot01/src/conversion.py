#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class Conver_vels():
    def __init__(self):
        self.sub = rospy.Subscriber('/cmd_vel',Twist,self.callback)
        self.right_wheel_pub = rospy.Publisher('/control_right_wheel/command',Float64)
        self.left_wheel_pub = rospy.Publisher('/control_left_wheel/command',Float64)
        self.cmd_vel = Twist()
        self.vr = Float64()
        self.vl = Float64()
        self.rate = rospy.Rate(50)

    def callback(self, msg):
        self.cmd_vel = msg
    
    def pub_wheel_vels(self):
        while not rospy.is_shutdown():
            self.convert_vels()
            self.right_wheel_pub.publish(self.vr)
            self.left_wheel_pub.publish(self.vl)
            self.rate.sleep()
    
    def convert_vels(self):
        # L = 0.3 //L is distance between wheels (wheel base)   //but now waiting for a true distance
        # R = 0.1 //R is radius                                 //but now waiting for a true distance
        
        self.vr = ((2*self.cmd_vel.linear.x) + (self.cmd_vel.angular.z*0.3))/(2*0.8)
        self.vl = ((2*self.cmd_vel.linear.x) - (self.cmd_vel.angular.z*0.3))/(2*0.8)
        

if __name__ == '__main__':
    rospy.init_node('convert_vels_node', anonymous=True)
    cv = Conver_vels()
    cv.pub_wheel_vels()

