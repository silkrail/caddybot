#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
import serial

def gps_publisher():
    rospy.init_node('gps_publisher', anonymous=True)
    pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)

    # 아두이노와의 직렬 포트 설정
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            try:
                # 데이터 파싱
                lat_str, lon_str, alt_str = data.split()
                latitude = float(lat_str.split(':')[1])
                longitude = float(lon_str.split(':')[1])
                altitude = float(alt_str.split(':')[1])
                
                # NavSatFix 메시지 생성
                gps_msg = NavSatFix()
                gps_msg.latitude = latitude
                gps_msg.longitude = longitude
                gps_msg.altitude = altitude
                gps_msg.header.stamp = rospy.Time.now()
                pub.publish(gps_msg)
                rospy.loginfo("Published GPS data: %s", data)
            except ValueError:
                rospy.logwarn("Failed to parse GPS data: %s", data)

        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass

