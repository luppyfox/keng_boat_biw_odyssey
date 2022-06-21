#! /usr/bin/env python3
import rospy 
import tf 
from sensor_msgs.msg import Imu 
import time
import board
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
from adafruit_lis3mdl import LIS3MDL
   
class IMU: 
	
	def __init__(self): 
		self.i2c = board.I2C()
		self.accel_gyro = LSM6DS33(i2c_bus=self.i2c, address=0x6A)
		self.mag = LIS3MDL(i2c_bus=self.i2c, address=0x1C)
	
		self.pub_imu = rospy.Publisher('/imu',Imu, queue_size=1)
		self.imu = Imu()
		
		self.rate = rospy.Rate(50) 
		self.updateImu()
	
	def updateImu(self): 
		while not rospy.is_shutdown(): 		
			self.imu.header.stamp = rospy.Time.now()
			self.imu.header.frame_id = "imu_link"			
			
			mag_x, mag_y, mag_z = self.mag.magnetic
			orien_quat = tf.transformations.quaternion_from_euler(mag_x, mag_y, mag_z)
			quat_x, quat_y, quat_z, quat_w = orien_quat
			self.imu.orientation.x = quat_x
			self.imu.orientation.y = quat_y
			self.imu.orientation.z = quat_z
			self.imu.orientation.w = quat_w
			self.imu.orientation_covariance[0] = -1
			
			gyro_x, gyro_y, gyro_z = self.accel_gyro.gyro
			self.imu.angular_velocity.x = gyro_x
			self.imu.angular_velocity.y = gyro_y
			self.imu.angular_velocity.z = gyro_z
			self.imu.angular_velocity_covariance[0] = -1
			
			acc_x, acc_y, acc_z = self.accel_gyro.acceleration
			self.imu.linear_acceleration.x = acc_x
			self.imu.linear_acceleration.y = acc_y
			self.imu.linear_acceleration.z = acc_z	
			self.imu.linear_acceleration_covariance[0] = -1
			
			self.rate.sleep()
			self.pub_imu.publish(self.imu)
            #end

if __name__ == "__main__":
	rospy.init_node('imu')
	imu = IMU()
