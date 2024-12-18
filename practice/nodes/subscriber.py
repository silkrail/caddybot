#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import PointStamped # 엔코더 메시지
import tf # tf broadcaster
from math import sin,cos,pi
from time import time
from nav_msgs.msg import Odometry # 오도메트리 메시지


 
 
def odom_callback(data) : 
     #print(odom_callback.count) 
     if (odom_callback.count % 50 == 0 ) :
         odom_callback.count += 1
         print("quat : ", (data.pose.pose.orientation.x))
         print("pose : ", (data.pose.pose.position.x))
         print("twist : ", (data.twist.twist.linear.x))
     odom_callback.count += 1
         



if __name__=="__main__":
    
    odom_callback.count = 0 
    rospy.init_node('subcriber_node')   
    rospy.Subscriber("/odom", Odometry, odom_callback)

    rate = rospy.Rate(50)
    #rospy.loginfo("publishing")
    rospy.spin()
    rate.sleep()


    
