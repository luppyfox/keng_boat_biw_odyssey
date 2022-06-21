#! /usr/bin/env python3

import rospy
from math import pi, sin, cos
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from std_msgs.msg import Float64, Int64
from nav_msgs.msg import Odometry

import tf
from tf.broadcaster import TransformBroadcaster

class OdometryClass:

    def __init__(self):
        self.left_ticks_sub = rospy.Subscriber('/Enc_L', Int64, self.getTicks_L)
        self.right_ticks_sub = rospy.Subscriber('/Enc_R', Int64, self.getTicks_R)
        
        self.vel_l_pub = rospy.Publisher('/enc_vel_l', Float64, queue_size=1)
        self.vel_r_pub = rospy.Publisher('/enc_vel_r', Float64, queue_size=1)

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.odom_broadcaster = TransformBroadcaster()
        self.odom = Odometry()
        self.rate = rospy.Rate(1)
        self.lastLeftTicks = 0
        self.lastRightTicks = 0
        self.currentLeftTicks = 0
        self.currentRightTicks = 0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.L = 0.42
        self.R = 0.06
        self.N = 40120

        self.vel_l = Float64()
        self.vel_r = Float64()      

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.updatePose()

    def getTicks_L(self, msg):
        self.currentLeftTicks = msg.data

    def getTicks_R(self, msg):
        self.currentRightTicks = msg.data
        
    def updatePose(self):

        while not rospy.is_shutdown():
            
            delta_l = self.currentLeftTicks - self.lastLeftTicks
            delta_r = self.currentRightTicks - self.lastRightTicks
            d_l = 2 * pi * self.R * delta_l / self.N
            d_r = 2 * pi * self.R * delta_r / self.N

            self.lastLeftTicks = self.currentLeftTicks
            self.lastRightTicks = self.currentRightTicks       
            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.last_time).to_sec()
            
            
            # d_l = (self.R * 2 * (pi / 60) * self.currentLeftTicks) * dt
            # d_r = (self.R * 2 * (pi / 60) * self.currentRightTicks) * dt   
            
            v = ((d_r + d_l) / 2) / dt
            w = ((d_r - d_l) / self.L) / dt

            
            self.vel_l = d_l / dt
            self.vel_r = d_r / dt

            self.vel_l_pub.publish(self.vel_l)
            self.vel_r_pub.publish(self.vel_r)

            # compute odometry in a typical way given the velocities of the robot
            delta_x = v * cos(self.th)
            delta_y = v * sin(self.th)
            delta_th = w

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                odom_quat,
                self.current_time,
                "base_footprint",
                "odom"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_footprint"
            odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))

            # publish the message
            self.odom_pub.publish(odom)

            self.last_time = self.current_time
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('pub_odom')
    oc = OdometryClass()
    rospy.spin()
    
