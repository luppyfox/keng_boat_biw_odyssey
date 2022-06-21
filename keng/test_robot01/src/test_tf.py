#!/usr/bin/env python3  
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('my_fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(3.0)
    while not rospy.is_shutdown():
        br.sendTransform( (1.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "laser",
                         "base")
        rate.sleep()
