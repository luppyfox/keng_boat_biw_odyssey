#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int64

rospy.init_node('pwm')
pwm_l = rospy.Publisher('/control_l',Int64,self.callback_l,queue_size=1)
pwm_r = rospy.Publisher('/control_r',Int64,self.callback_r,queue_size=1)
a = Int64()
b = Int64()
a.data = 65
b.data = 65

while not rospy.is_shutdown():
    pwm_l.publish(a)
    pwm_r.publish(b)
    rate.sleep()