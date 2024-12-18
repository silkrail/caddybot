import rospy
import csv
import time
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from datetime import datetime

# 데이터 저장 변수
buffer = []  # 데이터를 임시 저장할 버퍼
save_interval = 30  # 저장 간격 (초)
start_time = None

# 속도 명령 및 실제 속도
current_cmd_vel = None
current_odom = None

# IMU 데이터 콜백 함수
def imu_callback(data):
    global buffer, current_cmd_vel, current_odom
    timestamp = rospy.Time.now().to_sec()

    # 데이터를 버퍼에 추가
    buffer.append([
        timestamp,
        data.linear_acceleration.x,
        data.linear_acceleration.y,
        data.linear_acceleration.z,
        data.angular_velocity.x,
        data.angular_velocity.y,
        data.angular_velocity.z,
        current_cmd_vel.linear.x if current_cmd_vel else None,
        current_cmd_vel.angular.z if current_cmd_vel else None,
        current_odom.twist.twist.linear.x if current_odom else None,
        current_odom.twist.twist.angular.z if current_odom else None
    ])

    # 일정 시간이 지나면 데이터를 저장
    if time.time() - start_time >= save_interval:
        save_csv_file()

# cmd_vel 메시지 콜백 함수
def cmd_vel_callback(msg):
    global current_cmd_vel
    current_cmd_vel = msg

# odom 메시지 콜백 함수
def odom_callback(msg):
    global current_odom
    current_odom = msg

# 데이터 저장 함수
def save_csv_file():
    global buffer, start_time
    if not buffer:  # 버퍼가 비어 있으면 저장하지 않음
        return

    # 파일 이름 생성 (타임스탬프 포함)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_path = f"/home/amr/catkin_ws/imu_data_{timestamp}.csv"

    # CSV 파일 저장
    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["timestamp", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z", "cmd_vel_linear_x", "cmd_vel_angular_z", "odom_linear_x", "odom_angular_z"])
        writer.writerows(buffer)
    
    rospy.loginfo(f"Saved {len(buffer)} entries to {file_path}")
    buffer = []  # 버퍼 초기화
    start_time = time.time()  # 시작 시간 갱신

# ROS 노드 초기화
def imu_to_csv_periodic():
    global start_time
    rospy.init_node("imu_to_csv_periodic", anonymous=True)
    start_time = time.time()  # 시작 시간 설정

    rospy.Subscriber("/imu", Imu, imu_callback)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    rospy.loginfo("Collecting IMU data... Press Ctrl+C to stop.")
    rospy.spin()

    # 노드 종료 시 마지막 버퍼 저장
    save_csv_file()

if __name__ == "__main__":
    try:
        imu_to_csv_periodic()
    except rospy.ROSInterruptException:
        pass

