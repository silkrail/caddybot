#!/usr/bin/env python3
import rospy
from robot_ctrl.msg import DepthandDeg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class DepthAndDegreePlotter:
    def __init__(self):
        rospy.init_node('depth_and_degree_plotter', anonymous=True)

        # Initialize data storage
        self.times = []  # Store all time values
        self.depths = []  # Store all depth values
        self.degrees = []  # Store all degree values
        self.start_time = rospy.Time.now()

        # Subscribers
        rospy.Subscriber('/depth_and_deg', DepthandDeg, self.callback)

        # Setup plots
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))

        # Plot for Depth
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Depth (m)', color='tab:blue')
        self.line1, = self.ax1.plot([], [], 'b-', label='Depth (m)')
        self.ax1.legend(loc='upper left')

        # Plot for Degree
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Degree (°)', color='tab:red')
        self.line2, = self.ax2.plot([], [], 'r-', label='Degree (°)')
        self.ax2.legend(loc='upper left')

        # Animation function
        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, blit=True)

        # Register shutdown hook
        rospy.on_shutdown(self.save_plot)

    def init_plot(self):
        self.line1.set_data([], [])
        self.line2.set_data([], [])
        return self.line1, self.line2

    def callback(self, msg):
        # Get the current time from the start of the node
        current_time = rospy.Time.now() - self.start_time
        elapsed_time = current_time.to_sec()

        # Only record data up to 20 seconds
        #if elapsed_time <= 20:
        self.times.append(elapsed_time)

            # Append the incoming depth and degree values
        self.depths.append(msg.center_depth)  # Convert depth from mm to meters
        self.degrees.append(msg.deg)
        #else:
            #rospy.loginfo("Recording finished: 20 seconds have passed.")

    def update_plot(self, frame):
        if len(self.times) > 0:
            # Update Depth plot
            self.line1.set_data(self.times, self.depths)
            self.ax1.set_xlim(self.times[0], self.times[-1])
            self.ax1.set_ylim(min(self.depths) - 0.1, max(self.depths) + 0.1)

            # Update Degree plot
            self.line2.set_data(self.times, self.degrees)
            self.ax2.set_xlim(self.times[0], self.times[-1])
            self.ax2.set_ylim(min(self.degrees) - 1, max(self.degrees) + 1)

            # Get current axis limits and display them in the title
            x1_min, x1_max = self.ax1.get_xlim()
            y1_min, y1_max = self.ax1.get_ylim()
            self.ax1.set_title(f"Depth Over Time | X: [{x1_min:.2f}, {x1_max:.2f}], Y: [{y1_min:.2f}, {y1_max:.2f}]")

            x2_min, x2_max = self.ax2.get_xlim()
            y2_min, y2_max = self.ax2.get_ylim()
            self.ax2.set_title(f"Degree Over Time | X: [{x2_min:.2f}, {x2_max:.2f}], Y: [{y2_min:.2f}, {y2_max:.2f}]")

        return self.line1, self.line2

    def save_plot(self):
        # Save the figure when the node is shutting down
        self.fig.savefig('depth_and_degree_plot.png')
        rospy.loginfo("Plot saved to depth_and_degree_plot.png")

    def run(self):
        plt.tight_layout()
        plt.show()
        rospy.spin()

if __name__ == '__main__':
    plotter = DepthAndDegreePlotter()
    plotter.run()

