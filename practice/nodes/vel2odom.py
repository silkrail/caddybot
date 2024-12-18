#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist # cmd_vel 메시지
from nav_msgs.msg import Odometry   # 오도메트리 메시지
import numpy as np
import tf # tf broadcaster
from math import sin, cos, pi
from time import time

def init_odometry():
    odom = Odometry()

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_footprint"

    # 초기 위치
    odom.pose.pose.position.x = 0.0
    odom.pose.pose.position.y = 0.0
    odom.pose.pose.position.z = 0.0
    # 초기 오리엔테이션 (쿼터니언)
    odom.pose.pose.orientation.x = 0.0
    odom.pose.pose.orientation.y = 0.0
    odom.pose.pose.orientation.z = 0.0
    odom.pose.pose.orientation.w = 1.0

    # 속도 초기화
    odom.twist.twist.linear.x = 0.0
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.linear.z = 0.0
    odom.twist.twist.angular.x = 0.0
    odom.twist.twist.angular.y = 0.0
    odom.twist.twist.angular.z = 0.0

    odom_publisher.publish(odom)

def cmd_vel_callback(cmd_msg):
    global time_prev, linear_vel_prev, angular_vel_prev, x_pos, y_pos, theta, count

    # 시간 계산
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
    delta_ly = delta_s * sin(delta_theta)

    delta_x = delta_lx * cos(theta) - delta_ly * sin(theta)
    delta_y = delta_lx * sin(theta) + delta_ly * cos(theta)

    count += 1
    if count < 4:  # 초기화 단계에서는 이동을 무시
        delta_x = 0
        delta_y = 0
        delta_theta = 0.0

    # 위치와 각도 업데이트
    x_pos += delta_x
    y_pos += delta_y
    theta += delta_theta

    if theta >= pi * 2:
        theta -= pi * 2
    if theta <= -pi * 2:
        theta += pi * 2

    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)

    # 오도메트리 메시지 작성 및 퍼블리시
    odom = Odometry()

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_footprint"

    odom.pose.pose.position.x = x_pos
    odom.pose.pose.position.y = y_pos
    odom.pose.pose.position.z = 0.0

    odom.pose.pose.orientation.x = quaternion[0]
    odom.pose.pose.orientation.y = quaternion[1]
    odom.pose.pose.orientation.z = quaternion[2]
    odom.pose.pose.orientation.w = quaternion[3]

    odom.pose.covariance = [1e-3, 0, 0, 0, 0, 0,
                            0, 1e-3, 0, 0, 0, 0,
                            0, 0, 1e6, 0, 0, 0,
                            0, 0, 0, 1e6, 0, 0,
                            0, 0, 0, 0, 1e6, 0,
                            0, 0, 0, 0, 0, 1e3]

    odom.twist.twist.linear.x = linear_vel
    odom.twist.twist.angular.z = angular_vel

    odom.twist.covariance = [1e-3, 0, 0, 0, 0, 0,
                             0, 1e-3, 0, 0, 0, 0,
                             0, 0, 1e6, 0, 0, 0,
                             0, 0, 0, 1e3, 0, 0,
                             0, 0, 0, 0, 1e6, 0,
                             0, 0, 0, 0, 0, 1e3]

    odom_publisher.publish(odom)


if __name__=="__main__":
    
    rospy.init_node('vel2odom')
    odom_publisher = rospy.Publisher('/odom_enco', Odometry, queue_size=50)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    
    # 초기값 설정
    global time_prev, linear_vel_prev, angular_vel_prev, x_pos, y_pos, theta, count
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


    
