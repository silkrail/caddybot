#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <serial/serial.h>
#include <string>
#include <sstream>

serial::Serial ser;

void parseGPSData(const std::string &nmea_sentence, sensor_msgs::NavSatFix &fix_msg) {
    if (nmea_sentence.substr(0, 6) == "$GPGGA") {
        // NMEA 문장 파싱 로직 (생략)
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_publisher");
    ros::NodeHandle nh;

    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);

    // 시리얼 포트 설정
    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    ros::Rate loop_rate(5);  // 5Hz 주기로 GPS 데이터 수신

    while (ros::ok()) {
        if (ser.available()) {
            std::string gps_data = ser.readline();
            ROS_INFO_STREAM("Received GPS Data: " << gps_data);  // 수신된 GPS 데이터 로그 출력
            
            sensor_msgs::NavSatFix fix_msg;
            parseGPSData(gps_data, fix_msg);  // NMEA 데이터 파싱

            gps_pub.publish(fix_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

