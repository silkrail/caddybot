#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import threading
#linear actuator 에 voltage 명령 부여하는 노드
class SetDownPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('set_down_publisher')

        # Create Publisher
        self.pub = rospy.Publisher('set_down', Int32, queue_size=10)

        # Initialize variables
        self.current_value = 0
        self.lock = threading.Lock()
        self.running = True

        # Start the input thread
        self.input_thread = threading.Thread(target=self.get_input)
        self.input_thread.start()

    def get_input(self):
        rospy.loginfo("Enter integers to publish to the 'set_down' topic. Type 'exit' to quit.")
        while self.running and not rospy.is_shutdown():
            try:
                # Get input from the terminal
                user_input = input("Enter an integer: ")

                if user_input.lower() == 'exit':
                    rospy.loginfo("Exiting the publisher.")
                    self.running = False
                    rospy.signal_shutdown("User requested shutdown.")
                    break

                # Convert the input to Int32
                int_value = int(user_input)
                with self.lock:
                    self.current_value = int_value

            except ValueError:
                rospy.logwarn("Invalid input. Please enter a valid integer or type 'exit' to quit.")

    def publish_loop(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown() and self.running:
            with self.lock:
                msg = Int32(data=self.current_value)
            self.pub.publish(msg)
            rate.sleep()

    def shutdown(self):
        self.running = False
        self.input_thread.join()

def main():
    publisher = SetDownPublisher()
    try:
        publisher.publish_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        publisher.shutdown()

if __name__ == '__main__':
    main()
