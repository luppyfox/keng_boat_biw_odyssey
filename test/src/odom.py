#! /usr/bin/env python3
import rospy
from math import pi, sin, cos
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
import tf
from tf.broadcaster import TransformBroadcaster

class OdometryClass:
    def __init__(self):
            self.enc_l_sub = rospy.Subscriber('/Enc_L', Int64, self.callback_L)
            self.enc_r_sub = rospy.Subscriber('/Enc_R', Int64, self.callback_R)
            self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size = 1)
            self.odom = Odometry()
            self.rate = rospy.Rate(1)
            self.odom_broadcaster = TransformBroadcaster()
            self.lastL_ticks = 0
            self.lastR_ticks = 0
            self.currentL_ticks = 0
            self.currentR_ticks = 0
            self.current_time = rospy.Time.now()
            self.last_time = rospy.Time.now()
            self.L = 0.308 #distance between robot wheels
            self.R = 0.06 #radius of wheel
            self.N = 40120 #
            self.x = 0
            self.y = 0
            self.theta = 0
            self.updatePose()

    def callback_L(self, msg):
        self.currentL_ticks = msg.data

    def callback_R(self, msg):
        self.currentR_ticks = msg.data
        
    def updatePose(self):
        while not rospy.is_shutdown():
            delta_l = self.currentL_ticks - self.lastL_ticks
            delta_r = self.currentR_ticks - self.lastR_ticks
            d_l = 2 * pi * self.R * delta_l / self.N
            d_r = 2 * pi * self.R * delta_r / self.N
            
            self.lastL_ticks = self.currentL_ticks
            self.lastR_ticks = self.currentR_ticks
            
            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.last_time).to_sec()
            
            v = ((d_r + d_l) / 2) / dt
            w = ((d_r + d_l) / self.L) / dt

            delta_x = v * cos(self.theta)
            delta_y = v * sin(self.theta)
            delta_th = w

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_th

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

            self.odom_broadcaster.sendTransform((self.x, self.y, 0), odom_quat, self.current_time, "base_footprint", "odom")
            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "odom"

            odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*odom_quat))

            odom.child_frame_id = "base_footprint"
            odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))
            
            self.last_time = self.current_time
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('pub_odom')
    oc = OdometryClass()
    rospy.spin()

            


