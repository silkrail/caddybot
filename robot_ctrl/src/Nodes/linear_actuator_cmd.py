#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from robot_ctrl.msg import BestPath
from unitree_lidar_ros.msg import Routes, Route
import message_filters

class RoutesProcessor:
    def __init__(self):
        # 노드 초기화
        rospy.init_node('linear_actuator_cmd_node', anonymous=True)

        # 초기 target_index 설정 (예: 3)
        self.target_index = 3

        # /routes 토픽과 /best_path 토픽을 message_filters로 구독
        self.routes_sub = message_filters.Subscriber('/routes_topic', Routes)
        self.best_path_sub = message_filters.Subscriber('/best_path', BestPath)

        # ApproximateTimeSynchronizer 설정 (queue_size=10, slop=0.1초)
        ats = message_filters.ApproximateTimeSynchronizer([self.routes_sub, self.best_path_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.synced_callback)

        # act_cmd 퍼블리셔 생성
        self.act_cmd_pub = rospy.Publisher('act_cmd', Float32, queue_size=10)

        rospy.loginfo("RoutesProcessor node has started.")
        rospy.loginfo("Default target_index: %d", self.target_index)

    def synced_callback(self, routes_msg, best_path_msg):
        self.target_index = best_path_msg.data
        target_route_index1 = self.target_index * 3 - 2
        target_route_index2 = self.target_index * 3 - 1
        centroid_z1 = None
        centroid_z2 = None
        for route in routes_msg.routes:
            if route.index == target_route_index1:
                centroid_z1 = route.centroid.z
            elif route.index == target_route_index2:
                centroid_z2 = route.centroid.z
            if centroid_z1 is not None and centroid_z2 is not None:
                break
        if centroid_z1 is not None and centroid_z2 is not None:
            z_diff = centroid_z2 - centroid_z1 # interest point  1 과 2 사이의 높이차 만큼 linear actuator 하강
            if z_diff < 0:
                act_cmd_value = -z_diff 
            else:
                act_cmd_value = 0.0
        else:
            rospy.logwarn(
                "Target route indices %d or %d not found in /routes message.",
                target_route_index1,
                target_route_index2,
            )
            act_cmd_value = 0.0

        self.publish_act_cmd(act_cmd_value)

    def publish_act_cmd(self, height):
        act_cmd_msg = Float32()
        act_cmd_msg.data = height
        self.act_cmd_pub.publish(act_cmd_msg)
        rospy.loginfo("Published act_cmd: %.3f", height)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        processor = RoutesProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass
