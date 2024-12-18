#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import tf # tf broadcaster
from math import sin,cos,pi
from time import time
import numpy as np
from nav_msgs.msg import Odometry # 오도메트리 메시지
from sensor_msgs.msg import JointState # 바퀴 조인트 상태 메시지

def init_odometry() : 

    Odometry()
            
    Odometry().header.stamp = rospy.Time.now()
    Odometry().header.frame_id = "odom"
    Odometry().child_frame_id = "base_footprint"

    # robot's position in x,y, and z
    Odometry().pose.pose.position.x = 0.0
    Odometry().pose.pose.position.y = 0.0
    Odometry().pose.pose.position.z = 0.0
    # robot's heading in quaternion
    Odometry().pose.pose.orientation.x = 0.0
    Odometry().pose.pose.orientation.y = 0.0
    Odometry().pose.pose.orientation.z = 0.0
    Odometry().pose.pose.orientation.w = 0.0

    Odometry().pose.covariance[0] = 0.0
    Odometry().pose.covariance[7] = 0.0
    Odometry().pose.covariance[14] = 0.0
    Odometry().pose.covariance[21] = 0.0
    Odometry().pose.covariance[28] = 0.0
    Odometry().pose.covariance[35] = 0.0

    # linear speed from encoders
    Odometry().twist.twist.linear.x = 0.0
    Odometry().twist.twist.linear.y = 0.0
    Odometry().twist.twist.linear.z = 0.0
    Odometry().twist.twist.angular.x = 0.0
    Odometry().twist.twist.angular.y = 0.0
    Odometry().twist.twist.angular.z = 0.0

    Odometry().twist.covariance[0] = 0.0
    Odometry().twist.covariance[7] = 0.0
    Odometry().twist.covariance[14] = 0.0
    Odometry().twist.covariance[21] = 0.0
    Odometry().twist.covariance[28] = 0.0
    Odometry().twist.covariance[35] = 0.0

    odom_publisher.publish(Odometry())


def encoder_callback(data) : 

    #time_prev = 0
    global time_prev
    global linear_vel_prev
    global angular_vel_prev
    global x_pos
    global y_pos
    global theta
    global count
    
    time_now = time()
    delta_time = time_now - time_prev
    time_prev = time_now

    # 가우시안 오차의 표준 편차를 정의 (필요에 맞게 조절)
    linear_noise_std = 0.03  # 예: 선속도에 대한 표준편차
    angular_noise_std = 0.04  # 예: 각속도에 대한 표준편차
    
    linear_vel = cmd_msg.linear.x
    angular_vel = cmd_msg.angular.z
    
    linear_vel_noisy = linear_vel + np.random.normal(0, linear_noise_std)
    angular_vel_noisy = angular_vel + np.random.normal(0, angular_noise_std)
    
    delta_s = linear_vel_noisy * delta_time # delta_time 동안의 이동거리
    delta_theta = angular_vel_noisy * delta_time # small angle[rad]

    delta_lx = delta_s * cos(delta_theta)
    delta_ly = delta_s * sin(delta_theta) # 로컬 좌표계의 delta_x,y 계산

    delta_x = (delta_lx * cos(theta) - delta_ly * sin(theta))
    delta_y = (delta_lx * sin(theta) + delta_ly * cos(theta))
    # global 좌표계의 delta_x,y계산

    # 처음 몇 번의 값을 제외해줘야 초기 오도메트리가 0으로 계산됨
    count += 1
    if count < 4 : 
        delta_x = 0
        delta_y = 0
        delta_theta = 0.0
    else : 
        count = 10

    x_pos += delta_x
    y_pos += delta_y
    theta += delta_theta
    linear_vel_prev = linear_vel
    angular_vel_prev = angular_vel

    if theta >= pi * 2 : theta -= pi * 2
    if theta <= -pi * 2 : theta += pi * 2

    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)

    # ----------------odometry 퍼블리시---------------------------

    odom = Odometry()
            
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_footprint"

    # robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos
    odom.pose.pose.position.y = y_pos
    odom.pose.pose.position.z = 0.0
    # robot's heading in quaternion
    odom.pose.pose.orientation.x = quaternion[0]
    odom.pose.pose.orientation.y = quaternion[1]
    odom.pose.pose.orientation.z = quaternion[2]
    odom.pose.pose.orientation.w = quaternion[3]

    odom.pose.covariance[0] = 1e-3
    odom.pose.covariance[7] = 1e-3
    odom.pose.covariance[14] = 1e6
    odom.pose.covariance[21] = 1e6
    odom.pose.covariance[28] = 1e6
    odom.pose.covariance[35] = 1e3

    # linear speed from encoders
    odom.twist.twist.linear.x = linear_vel
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.linear.z = 0.0
    odom.twist.twist.angular.x = 0.0
    odom.twist.twist.angular.y = 0.0
    odom.twist.twist.angular.z = angular_vel

    odom.twist.covariance[0] = 1e-3
    odom.twist.covariance[7] = 1e-3
    odom.twist.covariance[14] = 1e6
    odom.twist.covariance[21] = 1e3
    odom.twist.covariance[28] = 1e6
    odom.twist.covariance[35] = 1e3

    odom_publisher.publish(odom)


if __name__=="__main__":
    
    rospy.init_node('publisher3')
    odom_publisher = rospy.Publisher('/odom_enco', Odometry, queue_size=50)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    
    global l_wheel_pulse
    global r_wheel_pulse
    global time_prev
    global linear_vel_prev
    global angular_vel_prev
    global x_pos
    global y_pos
    global theta
    global count
    time_prev = 0.0
    linear_vel_prev = 0.0
    angular_vel_prev = 0.0
    x_pos = 0.0
    y_pos = 0.0
    theta = 0.0
    count = 0

    init_odometry()

    rate = rospy.Rate(50)
    rospy.loginfo("publishing")
    rospy.spin()
    rate.sleep()


    
