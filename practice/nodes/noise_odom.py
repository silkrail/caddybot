import rospy
import random
from nav_msgs.msg import Odometry

def add_noise_to_odometry(odom):
    # Add Gaussian noise to position
    noise_x = random.gauss(0, 0.001)  # Mean = 0, StdDev = 0.001
    noise_y = random.gauss(0, 0.001)
    noise_z = random.gauss(0, 0.001)

    # Add Gaussian noise to orientation
    noise_orientation_x = random.gauss(0, 1e-5)
    noise_orientation_y = random.gauss(0, 1e-5)
    noise_orientation_z = random.gauss(0, 1e-5)
    noise_orientation_w = random.gauss(0, 1e-5)

    # Update the position
    odom.pose.pose.position.x += noise_x
    odom.pose.pose.position.y += noise_y
    odom.pose.pose.position.z += noise_z

    # Update the orientation
    odom.pose.pose.orientation.x += noise_orientation_x
    odom.pose.pose.orientation.y += noise_orientation_y
    odom.pose.pose.orientation.z += noise_orientation_z
    odom.pose.pose.orientation.w += noise_orientation_w

    # Add Gaussian noise to linear velocity
    odom.twist.twist.linear.x += random.gauss(0, 0.01)  # Mean = 0, StdDev = 1e-6
    odom.twist.twist.linear.y += random.gauss(0, 0.01)
    odom.twist.twist.linear.z += random.gauss(0, 0.01)

    # Add Gaussian noise to angular velocity
    odom.twist.twist.angular.x += random.gauss(0, 0.01)  # Mean = 0, StdDev = 1e-6
    odom.twist.twist.angular.y += random.gauss(0, 0.01)
    odom.twist.twist.angular.z += random.gauss(0, 0.01)

    return odom

def odometry_callback(odom_raw):
    # Add noise to the received odometry data
    noisy_odometry = add_noise_to_odometry(odom_raw)

    # Publish the noisy odometry
    odom_pub.publish(noisy_odometry)

def odometry_noisy_publisher():
    global odom_pub
    rospy.init_node('odometry_noisy_publisher', anonymous=True)
    odom_pub = rospy.Publisher('odom_noise', Odometry, queue_size=10)

    # Subscribe to the raw odometry data
    rospy.Subscriber('odom_enco', Odometry, odometry_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        odometry_noisy_publisher()
    except rospy.ROSInterruptException:
        pass

