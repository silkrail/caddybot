import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from tf.transformations import quaternion_matrix
import message_filters
from collections import deque
import time


class PointCloudDataCollector:
    def __init__(self):
        rospy.init_node('lidar_data_collector', anonymous=True)

        # PointCloud와 Odometry 메시지 필터로 동기화
        lidar_sub = message_filters.Subscriber('unilidar/cloud', PointCloud2)
        odom_sub = message_filters.Subscriber('odom', Odometry)

        # ApproximateTimeSynchronizer로 메시지 동기화
        self.ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, odom_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.lidar_odom_callback)

        # 데이터 버퍼 초기화
        self.data_buffer = deque(maxlen=100)  # 오래된 데이터를 자동으로 제거하며 저장
        self.time_buffer = deque(maxlen=100)  # 각 데이터의 타임스탬프를 저장
        self.corrected_pointcloud_pub = rospy.Publisher('corrected_pointcloud', PointCloud2, queue_size=10)

        self.buffer_duration = 0.5  # 유지할 데이터의 시간 길이 (2초)
        self.last_publish_time = time.time()  # 마지막 퍼블리시 시간 추적

    def lidar_odom_callback(self, pointcloud_msg, odom_msg):
        # 동기화된 PointCloud와 Odometry 데이터를 수집
        current_time = rospy.Time.now().to_sec()
        self.data_buffer.append((pointcloud_msg, odom_msg))
        self.time_buffer.append(current_time)

        # 오래된 데이터를 제거 (buffer_duration 기준으로 이전 데이터를 삭제)
        while self.time_buffer and (current_time - self.time_buffer[0] > self.buffer_duration):
            self.data_buffer.popleft()
            self.time_buffer.popleft()

    def process_and_publish(self):
        # 데이터가 없으면 처리하지 않음
        if not self.data_buffer:
            rospy.logwarn("No data in buffer to process")
            return

        all_points = []
        header = None

        for pointcloud_msg, odom_msg in list(self.data_buffer):  # 버퍼 복사본 사용
            points = []
            for point in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True):
                if -1 <= point[0] <= 1 and -3 <= point[1] <= 3 and point[2] <= 3:
                #if 0 <= point[0] <= 3 and -3 <= point[1] <= 3 and point[2] <= 1:
                    points.append([point[0], point[1], point[2]])

            if not points:
                rospy.logwarn("No valid points collected in the current frame")
                continue

            header = pointcloud_msg.header  # PointCloud 메시지의 헤더 저장
            header.frame_id = "odom"  # 프레임 ID를 odom으로 변경
            points = np.array(points)
            transformed_points = self.transform_to_world_coordinates(points, odom_msg)  # 월드 좌표계로 변환
            all_points.extend(transformed_points)

        if header and all_points:
            self.publish_corrected_pointcloud(all_points, header)
        rospy.loginfo("1")

    def transform_to_world_coordinates(self, points, odom_msg):
        # Odometry 정보에서 위치와 방향 추출
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation

        # 쿼터니언을 사용하여 로봇의 회전 행렬 생성
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        rotation_matrix = quaternion_matrix(quaternion)[:3, :3]  # 상위 3x3 회전 행렬만 사용

        # 변환된 포인트 저장
        transformed_points = []
        for point in points:
            # 포인트를 로봇의 위치와 회전에 따라 변환
            #point_robot = np.array([point[0] , point[1],point[2]])
            point_robot = np.array([point[2] + 0.36, point[1], 0.45 - point[0]])
            point_world = np.dot(rotation_matrix, point_robot) + np.array([position.x, position.y, position.z])
            transformed_points.append(point_world)

        return transformed_points

    def publish_corrected_pointcloud(self, points, header):
        # 보정된 PointCloud 퍼블리시
        corrected_cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.corrected_pointcloud_pub.publish(corrected_cloud)


if __name__ == '__main__':
    try:
        collector = PointCloudDataCollector()
        rate = rospy.Rate(30)  # 10 Hz 주기로 반복

        while not rospy.is_shutdown():
            current_time = time.time()

            # 1초에 한 번 퍼블리시
            if current_time - collector.last_publish_time >= 0.1:
                collector.process_and_publish()
                collector.last_publish_time = current_time
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

