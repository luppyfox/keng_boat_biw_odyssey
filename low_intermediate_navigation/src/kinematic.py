#! /usr/bin/env python3

import rospy
from math import pi
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class ConvertVelocities():
    def __init__(self):
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        self.vL_pub = rospy.Publisher('/control_left_wheel/command', Float32, queue_size=1)
        self.vR_pub = rospy.Publisher('/control_right_wheel/command', Float32, queue_size=1)
        self.cmd_vel = Twist()
        self.R = 0.06 #radius of wheel
        self.L = 0.308 #distance between wheels
        self.rate = rospy.Rate(1)

    def callback(self, msg):
        self.cmd_vel = msg

    def convert_vel(self):
        self.vl = self.cmd_vel.linear.x - (self.cmd_vel.angular.z*(self.L/2))
        self.vr = self.cmd_vel.linear.x + (self.cmd_vel.angular.z*(self.L/2))
        self.rpmL = (self.vl*60)/(2*pi*self.R)
        self.rpmR = (self.vr*60)/(2*pi*self.R)

        if self.rpmL >= 8:
            self.rpmL = 8

        if self.rpmR >= 8:
            self.rpmR = 8

        if self.rpmL <= -8:
            self.rpmL = -8

        if self.rpmR <= -8:
            self.rpmR = -8

    def pub_rpm(self):
        rospy.loginfo("Kuai")
        while not rospy.is_shutdown():
            self.convert_vel()
            self.vL_pub.publish(self.rpmL)
            self.vR_pub.publish(self.rpmR)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("kinematic")
    CV_object = ConvertVelocities()
    CV_object.pub_rpm()
