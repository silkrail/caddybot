#!/usr/bin/env python3
import rospy
#from practice.msg import coor 
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class LidarProcessor:
    def __init__(self):
        rospy.init_node('lidar_processor')
        
        self.lidar_sub = rospy.Subscriber('/lidar_scan', LaserScan, self.lidar_callback)
        self.pointcloud_pub = rospy.Publisher('/lidar_pointcloud', PointCloud2, queue_size=10)
        self.horizon_num = 40
        self.vertical_num = 40
        self.ver_range = 90
        
    def lidar_callback(self, data):

        ranges = np.array(data.ranges)
        angles = np.linspace(data.angle_max, data.angle_min, self.horizon_num)
        vngles = np.linspace(0, self.ver_range, self.vertical_num)
        new_angles = np.tile(angles, self.vertical_num)
        new_ver_angles = np.repeat(vngles, self.horizon_num)
        new_ranges = np.around(ranges, decimals=4)
       
            
        x_coords = 0.3 + new_ranges * np.cos(new_angles) * np.cos(np.radians(new_ver_angles))
        y_coords = - new_ranges * np.sin(new_angles) * np.cos(np.radians(new_ver_angles))
        z_coords = 0.45 - (new_ranges * np.sin(np.radians(new_ver_angles)))
        print(x_coords)
        n_x = np.around(x_coords, decimals=4)
        n_y = np.around(y_coords, decimals=4)
        n_z = np.around(z_coords, decimals=4)
        print(n_x)
        pointcloud_msg = PointCloud2()
        header = data.header
        pointcloud_msg.header = header
        
        pointcloud_msg.height = 1
        pointcloud_msg.width = len(n_x)
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.point_step = 12
        pointcloud_msg.row_step = pointcloud_msg.point_step * len(n_x)
        pointcloud_msg.is_dense = True
        
        pointcloud_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        points = []
        for i in range(len(n_x)):
            point = [n_x[i], n_y[i], n_z[i]]
            points.append(point)
            
        pointcloud_msg.data = np.array(points, dtype=np.float32).tobytes()
        self.pointcloud_pub.publish(pointcloud_msg)
        
if __name__ == '__main__':
    try:
        lidar_processor = LidarProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

