#!/usr/bin/env python3
import rospy
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tf
from robot_ctrl.msg import DepthandDeg

class DepthAndOdometryPlotter:
    def __init__(self):
        rospy.init_node('depth_and_odometry_plotter', anonymous=True)

        # Initialize data storage (no limit on points)
        self.x_world_coords = []  # 사람의 좌표
        self.y_world_coords = []
        self.x_cam_coords = []    # 카메라의 좌표
        self.y_cam_coords = []
        self.start_time = rospy.Time.now()

        # Initialize tf listener
        self.listener = tf.TransformListener()

        # Subscribers
        rospy.Subscriber('/depth_and_deg', DepthandDeg, self.depth_and_deg_callback)

        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_xlabel('X Coordinate (m)')
        self.ax.set_ylabel('Y Coordinate (m)')
        self.human_line, = self.ax.plot([], [], 'bo-', label='Human Position (m)')  # 파란색 사람 좌표
        self.camera_line, = self.ax.plot([], [], 'ro-', label='Camera Position (m)')  # 빨간색 카메라 좌표
        self.ax.legend(loc='upper left')
        self.ax.set_title("Human and Camera Positions in World Frame")

        # Set fixed limits for x and y axes
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)

        # Initialize animation with save_count to suppress warning
        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, blit=True, save_count=1000)



    def init_plot(self):
        self.human_line.set_data([], [])
        self.camera_line.set_data([], [])
        return self.human_line, self.camera_line

    def depth_and_deg_callback(self, msg):
        # Convert degree to radian (각도를 라디안으로 변환)
        rad = np.deg2rad(msg.deg)

        # 카메라 좌표계에서 사람의 좌표 (depth, degree 이용)
        x_cam = msg.center_depth * np.sin(rad)
        y_cam = msg.center_depth * np.cos(rad)
        try:
            now = rospy.Time.now()

            # odom -> cam_link로의 트랜스폼을 계산
            self.listener.waitForTransform("/odom", "/cam_link", now, rospy.Duration(0.1))
            
            # 카메라 좌표계에서의 변환된 위치를 구함
            (trans, rot) = self.listener.lookupTransform("/odom", "/cam_link", rospy.Time(0))
            # Quaternion을 Euler로 변환하여 Yaw(회전 각도) 추출
            quaternion = (rot[0], rot[1], rot[2], rot[3])
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]
            yaw +=(3 * np.pi) / 2

            # 카메라 좌표에서 얻은 x_cam, y_cam 값을 월드 좌표계로 변환
            cam_x_world = trans[0] + (x_cam * np.cos(yaw) - y_cam * np.sin(yaw))
            cam_y_world = trans[1] + (x_cam * np.sin(yaw) + y_cam * np.cos(yaw))
            # 월드 좌표계에서의 사람의 좌표 저장
            self.x_world_coords.append(cam_x_world)
            self.y_world_coords.append(cam_y_world)

            # 카메라 좌표 저장
            self.x_cam_coords.append(trans[0])
            self.y_cam_coords.append(trans[1])



        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"TF Error: {e}")

    def update_plot(self, frame):
        if len(self.x_world_coords) > 0:
            # Update position plot with world coordinates (Human)
            self.human_line.set_data(self.x_world_coords, self.y_world_coords)

        if len(self.x_cam_coords) > 0:
            # Update position plot with camera coordinates (Camera)
            self.camera_line.set_data(self.x_cam_coords, self.y_cam_coords)

        return self.human_line, self.camera_line



    def run(self):
        plt.tight_layout()
        plt.show()
        rospy.spin()

if __name__ == '__main__':
    plotter = DepthAndOdometryPlotter()
    plotter.run()

