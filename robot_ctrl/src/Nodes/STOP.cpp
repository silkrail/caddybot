#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv) {
    // ROS 초기화
    ros::init(argc, argv, "stop_node");
    ros::NodeHandle nh;

    // cmd_vel 토픽을 퍼블리시할 퍼블리셔 생성
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // 루프 주기 설정 (10Hz)
    ros::Rate loop_rate(10);

    // 멈추기 명령을 포함하는 Twist 메시지 생성
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.linear.y = 0.0;
    stop_msg.linear.z = 0.0;
    stop_msg.angular.x = 0.0;
    stop_msg.angular.y = 0.0;
    stop_msg.angular.z = 0.0;

    ROS_INFO("STOP node started, publishing zero velocity to cmd_vel");

    // 루프 실행
    while (ros::ok()) {
        // cmd_vel 토픽에 멈추기 명령 퍼블리시
        vel_pub.publish(stop_msg);
        // 루프 주기 유지
        loop_rate.sleep();
    }

    return 0;
}

