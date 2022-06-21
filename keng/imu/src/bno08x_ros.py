#! /usr/bin/env python3
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
import time
import rospy
from sensor_msgs.msg import Imu

import tf 
from tf.broadcaster import TransformBroadcaster


class IMU:
    def __init__(self):
        self.i2c = I2C(1)  # Device is /dev/i2c-2
        time.sleep(1)
        self.bno = BNO08X_I2C(self.i2c)

        self.pub_imu = rospy.Publisher('/imu', Imu, queue_size=1)
        self.imu = Imu()

        self.imu_tf_broadcaster = TransformBroadcaster()

        self.rate = rospy.Rate(50)
        self.enableImu()
        rospy.loginfo("BNO08X Start!!!")
        self.updateImu()

    def enableImu(self):
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

    def updateImu(self):
            self.imu.header.stamp = rospy.Time.now()
            self.imu.header.frame_id = "imu_link"

            quat_x, quat_y, quat_z, quat_w = self.bno.quaternion
            self.imu.orientation.x = quat_x
            self.imu.orientation.y = quat_y
            self.imu.orientation.z = quat_z
            self.imu.orientation.w = quat_w
            self.imu.orientation_covariance[0] = -1

            gyro_x, gyro_y, gyro_z = self.bno.gyro
            self.imu.angular_velocity.x = gyro_x
            self.imu.angular_velocity.y = gyro_y
            self.imu.angular_velocity.z = gyro_z
            self.imu.angular_velocity_covariance[0] = -1

            acc_x, acc_y, acc_z = self.bno.acceleration
            self.imu.linear_acceleration.x = acc_x
            self.imu.linear_acceleration.y = acc_y
            self.imu.linear_acceleration.z = acc_z
            self.imu.linear_acceleration_covariance[0] = -1

            self.rate.sleep()
            self.pub_imu.publish(self.imu)

            self.imu_tf_broadcaster.sendTransform(
                (0, 0, 0),
                self.bno.quaternion,
                rospy.Time.now(),
                "imu_link",
                "base_link"
            )

if __name__ == "__main__":
    rospy.init_node('imu_bno08x')
    imu = IMU()
