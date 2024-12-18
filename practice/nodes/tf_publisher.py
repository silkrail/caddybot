#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import tf # tf broadcaster

from nav_msgs.msg import Odometry # 오도메트리 메시지



def odometry_callback(data) : 


    br = tf.TransformBroadcaster()
    br.sendTransform((data.pose.pose.position.x , data.pose.pose.position.y , 0),
                     (data.pose.pose.orientation.x , data.pose.pose.orientation.y , data.pose.pose.orientation.z , data.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "base_footprint",
                     "odom")


if __name__=="__main__":
    
    rospy.init_node('tf_publisher')   
    rospy.Subscriber("/odom", Odometry, odometry_callback)


    rate = rospy.Rate(100)
    rospy.loginfo("publishing tf of odometry_filtered")
    rospy.spin()
    rate.sleep()


    
