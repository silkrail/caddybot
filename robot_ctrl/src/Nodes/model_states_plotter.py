#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from gazebo_msgs.msg import ModelStates
#gazebo 상의 actor 모델의 실제 위치 plotter
class ActorPositionPlotter:
    def __init__(self):
        rospy.init_node('actor_position_plotter', anonymous=True)

        # Initialize data storage (no limit on points)
        self.x_world_coords = []
        self.y_world_coords = []
        self.actor_name = rospy.get_param('~actor_name', 'walking_person')

        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_xlabel('X Coordinate (m)')
        self.ax.set_ylabel('Y Coordinate (m)')
        self.line, = self.ax.plot([], [], 'bo-', label=f'{self.actor_name} Position (m)')
        self.ax.legend(loc='upper left')
        self.ax.set_title(f"{self.actor_name} Position in World Frame")

        # Set fixed limits for x and y axes
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)

        # Initialize animation
        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, interval=100, blit=True)

        # Subscribe to /gazebo/model_states
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

    def init_plot(self):
        self.line.set_data([], [])
        return self.line,

    def model_states_callback(self, msg):
        # Check if the actor exists in the list of models
        if self.actor_name in msg.name:
            # Get the index of the actor
            index = msg.name.index(self.actor_name)

            # Get the position of the actor
            actor_pose = msg.pose[index].position

            # Append the actor's position to the list
            self.x_world_coords.append(actor_pose.x)
            self.y_world_coords.append(actor_pose.y)
        else:
            rospy.logwarn(f"Actor '{self.actor_name}' not found in /gazebo/model_states")

    def update_plot(self, frame):
        # Update plot with current world coordinates
        self.line.set_data(self.x_world_coords, self.y_world_coords)
        return self.line,

    def run(self):
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    plotter = ActorPositionPlotter()
    plotter.run()

