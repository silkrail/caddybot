#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from math import pow
from sensor_msgs.msg import Imu # 바퀴 조인트 상태 메시지


def imu_callback(data) : 

    global orientation_x_b
    global orientation_y_b
    global orientation_z_b
    global orientation_w_b
    global orientation_filtered_x
    global orientation_filtered_y
    global orientation_filtered_z
    global orientation_filtered_w

    global gyro_x_b
    global gyro_y_b
    global gyro_z_b
    global gyro_filtered_x
    global gyro_filtered_y
    global gyro_filtered_z

    global accel_x_b
    global accel_y_b
    global accel_z_b
    global accel_filtered_x
    global accel_filtered_y
    global accel_filtered_z

    global count

    
    imu = Imu()

    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = data.header.frame_id

    imu.orientation_covariance = data.orientation_covariance
    imu.angular_velocity_covariance = data.angular_velocity_covariance
    imu.linear_acceleration_covariance = data.linear_acceleration_covariance

#-------------------------------------------------------

    
    quat_1 = (data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)
    
    euler_angle = euler_from_quaternion(quat_1)

    quat = quaternion_from_euler(euler_angle[0], -euler_angle[1], -euler_angle[2])

    imu.orientation.x = quat[0]
    imu.orientation.y = quat[1]
    imu.orientation.z = quat[2]
    imu.orientation.w = quat[3]
#-------------------------------------------------------

    imu.angular_velocity.x = data.angular_velocity.x
    imu.angular_velocity.y = data.angular_velocity.y
    imu.angular_velocity.z = data.angular_velocity.z

    imu.linear_acceleration.x = data.linear_acceleration.x
    imu.linear_acceleration.y = data.linear_acceleration.y
    imu.linear_acceleration.z = data.linear_acceleration.z

    imu_publisher.publish(imu)
#-----------------------------------------------------------------------------------------

if __name__=="__main__":
    
    rospy.init_node('imu_filter')
    rospy.Subscriber("/imu", Imu, imu_callback)
    imu_publisher = rospy.Publisher('/imu_data', Imu, queue_size=50)


    global orientation_x_b
    global orientation_y_b
    global orientation_z_b
    global orientation_w_b
    global orientation_filtered_x
    global orientation_filtered_y
    global orientation_filtered_z
    global orientation_filtered_w

    global gyro_x_b
    global gyro_y_b
    global gyro_z_b
    global gyro_filtered_x
    global gyro_filtered_y
    global gyro_filtered_z

    global accel_x_b
    global accel_y_b
    global accel_z_b
    global accel_filtered_x
    global accel_filtered_y
    global accel_filtered_z

    global count

    orientation_x_b = 0.0
    orientation_y_b = 0.0
    orientation_z_b = 0.0
    orientation_w_b = 0.0
    orientation_filtered_x = 0.0
    orientation_filtered_y = 0.0
    orientation_filtered_z = 0.0
    orientation_filtered_w = 0.0

    gyro_x_b = 0.0
    gyro_y_b = 0.0
    gyro_z_b = 0.0
    gyro_filtered_x = 0.0
    gyro_filtered_y = 0.0
    gyro_filtered_z = 0.0

    accel_x_b = 0.0
    accel_y_b = 0.0
    accel_z_b = 9.81
    accel_filtered_x = 0.0
    accel_filtered_y = 0.0
    accel_filtered_z = 0.0

    count = 0


    rate = rospy.Rate(50)
    rospy.loginfo("publishing new imu data")
    rospy.spin()
    rate.sleep()


