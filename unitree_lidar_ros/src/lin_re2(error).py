import numpy as np
import rospy
import math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_matrix
from unitree_lidar_ros.msg import PathScore, LenWay, Route, Routes
from geometry_msgs.msg import Point

class SlopeEstimator:
    def __init__(self):
        rospy.init_node('pointcloud_slope_estimator', anonymous=True)
        self.current_odom = None
        self.current_pointcloud = None  # PointCloud 데이터를 저장할 변수
        
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        # routes_pub를 ROS Publisher로 정의
        self.routes_pub = rospy.Publisher('/routes_topic', Routes, queue_size=10)

        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/corrected_pointcloud', PointCloud2, self.pointcloud_callback)
        # 주기적으로 pointcloud를 처리하는 타이머
        self.timer = rospy.Timer(rospy.Duration(0.01), self.process_pointcloud)
        self.a = 1
        self.b = 1
        self.c = 1
        
    def odom_callback(self, msg):
        self.current_odom = msg
        self.current_z = msg.pose.pose.position.z

    def pointcloud_callback(self, pointcloud):
        self.current_pointcloud = pointcloud

    def process_pointcloud(self, event):
        # Odometry와 PointCloud 데이터가 모두 존재해야 처리
        if self.current_odom is None or self.current_pointcloud is None:
            #rospy.logwarn("Waiting for both odometry and pointcloud data.")
            return
        x = 0
        y = 0
        z = 0
        set_angle = [0.7,0.47,0.24,0,-0.24,-0.47,-0.7]
        set_time = [1,1.5,2]
        interest_points=[]
        for w in set_angle:
            for t in set_time:
                r = 1 / w if w != 0 else float('inf')  # w가 0일 경우 무한대 처리
                if r != float('inf'):
                    x = r * math.sin(w * t)
                    y = r - r * math.cos(w * t)
                    interest_points.append(np.array([x, y, z]))
                else:
                    # w가 0일 경우에는 x, y를 특정값으로 처리
                    x, y = t, 0
                    interest_points.append(np.array([x, y, z]))
        interest_points = [point - np.array([0.0, 0, 0]) for point in interest_points]
        

        current_position = self.current_odom.pose.pose.position
        current_orientation = self.current_odom.pose.pose.orientation
        current_orientation_list = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        
        rotation_matrix = quaternion_matrix(current_orientation_list)[:3, :3]  # 상위 3x3 회전 행렬만 사용
        
        # 위치 변화량 벡터 (delta_x, delta_y, delta_z) 생성
        translation_vector = np.array([current_position.x, current_position.y , current_position.z])
        routes = []
        # 각 관심 지점에 대해 평행 이동 및 회전 적용
        init_centroid = [self.current_odom.pose.pose.position.x,self.current_odom.pose.pose.position.y,self.current_odom.pose.pose.position.z]
        for i, interest_point in enumerate(interest_points):
            
            # 포인트 회전 보정
            rotated_interest_point = np.dot(rotation_matrix, interest_point)
            # 관심 지점에 대한 평행 이동
            moved_point = rotated_interest_point + translation_vector
    
            # 각 관심 지점에 대해 탐지반지름 내의 포인트를 수집 (z 좌표 무시)
            points = []
            for point in pc2.read_points(self.current_pointcloud, skip_nans=True):
                point_np = np.array([point[0], point[1], point[2]])
                if i in [2, 5, 8, 11, 14, 17, 20]:
                    radius = 0.3  # 탐지반지름
                else:
                    radius = 0.2  # 탐지반지름
                # x, y 좌표가 radius 범위 내에 있는지 확인
                if (moved_point[0] - radius <= point_np[0] <= moved_point[0] + radius and
                    moved_point[1] - radius <= point_np[1] <= moved_point[1] + radius):
                    points.append(point_np)

            points = np.array(points)
            # 각 관심 지점에 대한 기울기 계산
            centroid, normal_vector = self.calculate_slope(points, moved_point)
            if normal_vector is not None and centroid is not None:
                routes.append([i + 1, centroid, normal_vector])
                self.publish_marker(centroid, normal_vector, self.marker_pub, i)
        routes.append([22,init_centroid,[0,0,1]])

        routes_msg = Routes()
        routes_msg.header.stamp = rospy.Time.now()  
        routes_msg.header.frame_id = "odom"  
        for idx, route in enumerate(routes):
            route_msg = Route()
            route_msg.index = route[0]
            route_msg.centroid.x = route[1][0]
            route_msg.centroid.y = route[1][1]
            route_msg.centroid.z = route[1][2]
            route_msg.normal_vector.x = route[2][0]
            route_msg.normal_vector.y = route[2][1]
            route_msg.normal_vector.z = route[2][2]
        
            # 'route1', 'route2', ... 형식으로 경로 이름 설정
            #route_name = f"route{idx+1}"
            #rospy.loginfo(f"Publishing {route_name}: Centroid - {route_msg.centroid}, Normal Vector - {route_msg.normal_vector}")
        
            routes_msg.routes.append(route_msg)  # 경로를 Routes 메시지에 추가

        self.routes_pub.publish(routes_msg) 
        self.evaluate_route(routes)
        #rospy.loginfo('time')
        
    def evaluate_route(self, routes):
        total_z_difference = 0.0  # Z 차이의 합
        cos_vector_n = 0.0         # 각도의 코사인 값
        cos_limit = math.cos(math.radians(45)) # 각도변화의 한계값
        classified_routes = {
        'way1': {'routes': [], 'score': 0, 'obstacle' :0},
        'way2': {'routes': [], 'score': 0, 'obstacle' :0},
        'way3': {'routes': [], 'score': 0, 'obstacle' :0},
        'way4': {'routes': [], 'score': 0, 'obstacle' :0},
        'way5': {'routes': [], 'score': 0, 'obstacle' :0},
        'way6': {'routes': [], 'score': 0, 'obstacle' :0},
        'way7': {'routes': [], 'score': 0, 'obstacle' :0}
        }

        for route in routes:
            index = route[0]
        
            if 1 <= index <= 3:
                classified_routes['way1']['routes'].append(route)
            elif 4 <= index <= 6:
                classified_routes['way2']['routes'].append(route)
            elif 7 <= index <= 9:
                classified_routes['way3']['routes'].append(route)
            elif 10 <= index <= 12:
                classified_routes['way4']['routes'].append(route)
            elif 13 <= index <= 15:
                classified_routes['way5']['routes'].append(route)
            elif 16 <= index <= 18:
                classified_routes['way6']['routes'].append(route)
            elif 19 <= index <= 21:
                classified_routes['way7']['routes'].append(route)
        

        path_scores = []  # 이름 변경: way_scores -> path_scores
        test_path = []
        len_ways = [] 
        for way, data in classified_routes.items():
            route_list = data['routes']
            
            if len(route_list) == 3:
                len_ways.append(len(route_list))
                z_differences = []
                for i in range(len(route_list)-1):
                    z_diff = route_list[i + 1][1][2] - route_list[i][1][2]
                    z_differences.append(z_diff)
                first_z_diff = abs(route_list[0][1][2] - self.current_odom.pose.pose.position.z)
                total_z_difference = first_z_diff + sum(z_differences)
                data['score'] += self.a * total_z_difference      # 제 1요인 지면의 높이 변화
                
                normal_sum = np.array([0, 0, 0]) 
                ang_difference = []
                for i in range(len(route_list) - 1):
                    normal_sum = normal_sum + route_list[i][2]
                    dot_vector = np.dot(route_list[i][2], route_list[i + 1][2])
                    size_vector = np.linalg.norm(route_list[i][2]) * np.linalg.norm(route_list[i + 1][2])
                    cos_vector = dot_vector / size_vector
                    if cos_vector < cos_limit:
                        data['score'] = 100
                        data['obstacle'] = 1
                    else:
                        data['score'] += self.b * 1/cos_vector    # 제 2요인 지면의 기울기 변화
                        
                normal_sum = normal_sum / len(route_list)
                dot_vector_n = np.dot(normal_sum, [0, 0, 1])
                size_vector_n = np.linalg.norm(normal_sum) * np.linalg.norm([0, 0, 1])
                cos_vector_n = dot_vector_n / size_vector_n
                if cos_vector_n < cos_limit:
                    data['score'] = 100
                    data['obstacle'] = 1
                else:
                    data['score'] += self.c * 1/cos_vector_n   # 제 3요인 지면의 전반적인 기울기                                    
            elif len(route_list) == 2:
                len_ways.append(len(route_list))
                data['score'] = 100
                data['obstacle'] = 1
                rospy.loginfo('lack of useable points')
            elif len(route_list) == 1:
                len_ways.append(len(route_list))
                data['score'] = 100
                data['obstacle'] = 1
                rospy.loginfo('lack of useable points') 
            elif len(route_list) == 0:
                len_ways.append(len(route_list))
                data['score'] = 100
                data['obstacle'] = 1
                rospy.loginfo('No useable point')

        for i in range(7):
            key = f'way{i+1}' 
            if classified_routes[key]['obstacle'] == 1:
                print('obstacle detected')
                if i != 0:
                    classified_routes[f'way{i}']['score'] =100
                if i !=6:
                    classified_routes[f'way{i+2}']['score'] =100
                    
        for i in range(7):
            key = f'way{i+1}'             
            path_scores.append(classified_routes[key]['score'])
            test_path.append([key,classified_routes[key]['score']])
        # test_path를 로깅
        rospy.logwarn("test_path: %s", str(test_path))
        
        # 7개의 성분을 가진 리스트로 만든 후 ROS 메시지 발행
        path_score_msg = PathScore()  # WayScore 메시지 객체 생성
        path_score_msg.score = path_scores  # path_scores 리스트를 score 필드에 넣음
        score_pub = rospy.Publisher('/path_score', PathScore, queue_size=10)
        score_pub.publish(path_score_msg)
        
        len_way_msg = LenWay()  # LenWay 메시지 객체 생성
        len_way_msg.len = len_ways  
        len_pub = rospy.Publisher('/len_way', LenWay, queue_size=10)
        len_pub.publish(len_way_msg)
        return classified_routes
        
            
    def calculate_slope(self, points, interest_point):
        if points.shape[0] < 10:
            #rospy.loginfo(f"Not enough points to estimate a plane at {interest_point}.")
            return None, None
        
        # 평면 추정 (최소 제곱법)
        centroid = np.mean(points, axis=0)
        centered_points = points - centroid
        _, _, vh = np.linalg.svd(centered_points)
        normal_vector = vh[-1]  # 평면의 법선 벡터

        # 기준 벡터를 기준으로 법선 벡터 정규화
        normal_vector = self.normalize_normal_vector(normal_vector)

        return centroid, normal_vector

    def normalize_normal_vector(self, normal_vector, reference_vector=np.array([0, 0, 1])):
        # 기준 벡터와의 내적을 통해 방향 확인
        if np.dot(normal_vector, reference_vector) < 0:
            # 반대 방향이면 벡터를 반전시킴
            normal_vector = -normal_vector
        return normal_vector

    def publish_marker(self, centroid, normal_vector, marker_pub, i):
        # Marker 생성
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "slope_markers"
        marker.id = i
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # 화살표 시작점 (점의 좌표)
        start_point = Point()
        start_point.x = centroid[0]
        start_point.y = centroid[1]
        start_point.z = centroid[2]
        
        # 화살표 끝점 (점의 좌표 + 벡터)
        end_point = Point()
        end_point.x = centroid[0] + normal_vector[0] / 5
        end_point.y = centroid[1] + normal_vector[1] / 5
        end_point.z = centroid[2] + normal_vector[2] / 5
        
        # 화살표의 시작점과 끝점 설정
        marker.points.append(start_point)
        marker.points.append(end_point)
        
        # Marker의 크기 설정
        marker.scale.x = 0.025  # Arrow의 길이
        marker.scale.y = 0.05  # Arrow의 두께
        marker.scale.z = 0.05  # Arrow의 두께

        # Marker 색상 설정
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Marker Publish
        marker_pub.publish(marker)

def main():
    slope_estimator = SlopeEstimator()
    rospy.spin()

if __name__ == '__main__':
    main()

