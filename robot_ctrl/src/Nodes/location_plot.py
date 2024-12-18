#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from robot_ctrl.msg import DepthandDeg

class DepthOnlyPlotter:
    def __init__(self):
        rospy.init_node('depth_only_plotter', anonymous=True)

        # Initialize data storage
        self.x_coords = []
        self.y_coords = []

        # Subscriber
        rospy.Subscriber('/depth_and_deg', DepthandDeg, self.depth_and_deg_callback)

        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_xlabel('X Coordinate (m)')
        self.ax.set_ylabel('Y Coordinate (m)')
        self.line, = self.ax.plot([], [], 'bo-', label='Human Position (m)')
        self.ax.legend(loc='upper left')
        self.ax.set_title("Human Positions")

        # Set fixed limits for x and y axes (필요에 따라 조정)
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)

        # Initialize animation
        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, blit=True, interval=100)

    def init_plot(self):
        self.line.set_data([], [])
        return self.line,

    def depth_and_deg_callback(self, msg):
        # Convert degree to radian
        rad = np.deg2rad(msg.deg)

        # 단순 polar to Cartesian 변환
        x = msg.center_depth * np.sin(rad)
        y = msg.center_depth * np.cos(rad)

        # 계산된 값을 리스트에 추가
        self.x_coords.append(x)
        self.y_coords.append(y)

    def update_plot(self, frame):
        if len(self.x_coords) > 0:
            self.line.set_data(self.x_coords, self.y_coords)
        return self.line,

    def run(self):
        plt.tight_layout()
        plt.show()
        rospy.spin()

if __name__ == '__main__':
    plotter = DepthOnlyPlotter()
    plotter.run()
