#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <robot_ctrl/DepthandDeg.h>
#include <unitree_lidar_ros/PathScore.h>
#include <unitree_lidar_ros/LenWay.h> 
#include <robot_ctrl/BestPath.h>
#include <tf/tf.h>
#include <math.h>
#include <limits>
#include <vector> 

using namespace std;

class PathSelector {
public:
    PathSelector() {
        // 퍼블리셔와 서브스크라이버 설정
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        target_pub = nh.advertise<geometry_msgs::Point>("/target_position", 10);
        best_path_pub = nh.advertise<robot_ctrl::BestPath>("/best_path", 10);

        odom_sub = nh.subscribe("/odom", 10, &PathSelector::odom_callback, this);
        depth_and_deg_sub = nh.subscribe("/depth_and_deg", 10, &PathSelector::depth_and_deg_callback, this);
        score_sub = nh.subscribe("/path_score", 10, &PathSelector::score_callback, this);
        len_way_sub = nh.subscribe("/len_way", 10, &PathSelector::len_way_callback, this);  // /len_way 구독 추가
        update_timer = nh.createTimer(ros::Duration(0.01), &PathSelector::update_target_position, this);
        initial_yaw_offset = 0.0;
        is_initial_yaw_set = false;

        // 변수 초기화
        linear_velocity = 0.1;          
        fixed_linear_velocity = 0.4;    
        robot_position = {0.0, 0.0};
        robot_heading = 0.0;
        desired_dis = 2.5;
        curr_e_dis = 0.0;
        prev_e_dis = 0.0;
        ddis = 0.0;
        kp = 1.0;
        kd = 0.1;
        emerg_flag = false;
        is_initialized = false;
        close_person = false;
        prev_position = {0.0, 0.0};
        curr_position = {0.0, 0.0};
        curr_pos = {0.0, 0.0};
        dx = 0.0;
        dy = 0.0;
        latest_center_depth = 0.0;
        last_callback_time = ros::Time::now();
        start_time = ros::Time::now();

        for (int i = 0; i < 7; i++) {
            external_scores[i] = 0.0;
        }
    }
    ros::Time getLastCallbackTime() const { return last_callback_time; }
    void setLinearVelocity(double velocity) { linear_velocity = velocity; }
    void stop_robot() {
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        cmd_vel_pub.publish(stop_msg);
    }

    void choose_best_path() {
        int best_path = -1;
        double min_score = std::numeric_limits<double>::infinity();
        double path_time = 1.0;  // 경로 평가 시간

        double target_x = target_position.x;
        double target_y = target_position.y;


        double angular_velocity_ratios[7] = {0.7, 0.47, 0.24, 0.0, -0.24, -0.47, -0.7};


        double distance_weight = 0.4;
        double angle_weight = 0.6;
        for (int i = 0; i < 7; i++) {
            double angular_velocity = angular_velocity_ratios[i];
            double end_x, end_y;
            calculate_end_point(angular_velocity, path_time, end_x, end_y);

            double target_angle = atan2(target_y - end_y, target_x - end_x);
            double path_angle = atan2(end_y - robot_position[1], end_x - robot_position[0]);           
            double angle_diff = fabs(target_angle - path_angle);


            angle_diff = fmod(angle_diff + M_PI, 2 * M_PI) - M_PI;
            angle_diff = fabs(angle_diff);

            double distance = sqrt(pow(end_x - target_x, 2) + pow(end_y - target_y, 2));
            double score = (distance * distance_weight) + (angle_diff * angle_weight);

            score += external_scores[i] * 0.9;
            ROS_INFO("score :%.5f", score);
            if (score < min_score) {
                min_score = score;
                best_path = i + 1;
            }
        }

        //ROS_INFO("Current center_depth: %.2f meters", latest_center_depth);
        //ROS_INFO("best_path :%d ", best_path);
        ROS_INFO("tx : %.2f , ty : %.2f ", target_position.x , target_position.y);
        
        double selected_angular_velocity = 0.0;

        if ((ros::Time::now() - start_time).toSec() <= 0.5) {
            selected_angular_velocity = 0.0; // 시작후 0.5초간 각속도 0
        } else if (best_path != -1) {
            robot_ctrl::BestPath best_path_msg;
            best_path_msg.header.stamp = ros::Time::now();
            best_path_msg.header.frame_id = "odom";
            best_path_msg.data = best_path;
            best_path_pub.publish(best_path_msg);

            if (linear_velocity < 0.4) {
                if (!close_person) {
                    selected_angular_velocity = fixed_linear_velocity * angular_velocity_ratios[best_path - 1];
                } else {
                    stop_robot();
                    close_person = false;
                }
            } else {
                selected_angular_velocity = linear_velocity * angular_velocity_ratios[best_path - 1];
            }
        } else {
            ROS_WARN("No valid path found.");
        }

        move_robot(selected_angular_velocity);
    }

private:
    // ROS 노드 핸들러 및 퍼블리셔/서브스크라이버
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Publisher target_pub;
    ros::Publisher best_path_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber depth_and_deg_sub;
    ros::Subscriber score_sub;
    ros::Subscriber len_way_sub;  
    ros::Timer update_timer;
    ros::Time start_time;    
    double linear_velocity;
    double fixed_linear_velocity;
    geometry_msgs::Point target_position;
    vector<double> robot_position;
    double robot_heading;
    double desired_dis;
    double curr_e_dis;
    double prev_e_dis;
    double ddis;
    double kp;
    double kd;
    vector<double> curr_pos;
    bool emerg_flag;
    bool is_initialized;
    bool close_person;

    vector<double> prev_position;
    vector<double> curr_position;
    float dx;
    float dy;

    double initial_yaw_offset;
    bool is_initial_yaw_set;

    double latest_center_depth;
    double external_scores[7];
    ros::Time last_callback_time;
    //len_way 메시지 구독
    void len_way_callback(const unitree_lidar_ros::LenWay::ConstPtr& msg) { 
        if (msg->len.size() == 7) { 
            int count = 0;
            for (int i = 0; i < 7; i++) {
                if (msg->len[i] == 1) { 
                    count ++;
                    }
            }
            //interest point 가 한개만 찍힌 경로가 3개 이상일 경우 비상 정지        
            if (count >= 3){        
                    ROS_WARN("Emergency stop triggered by len_way message.");
                    stop_robot();
                    emerg_flag = true;
                }
            }
         else {
            ROS_WARN("Received len_way message with incorrect size.");
        }
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        robot_position[0] = msg->pose.pose.position.x;
        robot_position[1] = msg->pose.pose.position.y;
        

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        if (!is_initial_yaw_set) {
            initial_yaw_offset = 0.0;
            is_initial_yaw_set = true;
        }

        robot_heading = yaw + initial_yaw_offset;
        robot_heading = atan2(sin(robot_heading), cos(robot_heading));
    }
    //카메라로 부터 depth_and_deg 메시지 구독
    void depth_and_deg_callback(const robot_ctrl::DepthandDeg::ConstPtr& msg) {
        last_callback_time = ros::Time::now();

        double center_depth = msg->center_depth;
        double degree = msg->deg;

        double angle_rad = degree * M_PI / 180.0;
        angle_rad = -angle_rad;
        //로봇의 좌표 world 프레임으로 전환
        double world_x = robot_position[0] + center_depth * cos(robot_heading + angle_rad);
        double world_y = robot_position[1] + center_depth * sin(robot_heading + angle_rad);
        curr_position[0] = world_x;
        curr_position[1] = world_y;
        curr_pos[0] = curr_position[0];
        curr_pos[1] = curr_position[1];

     

        dx = curr_position[0] - prev_position[0];
        dy = curr_position[1] - prev_position[1];
        if(!is_initialized){
            prev_position[0] = curr_position[0];
            prev_position[1] = curr_position[1];
            dx = 0;
            dy = 0;
            is_initialized = true;
            }
        prev_position[0] = curr_position[0];
        prev_position[1] = curr_position[1];




        curr_e_dis = center_depth - desired_dis;
        ddis = curr_e_dis - prev_e_dis;
        prev_e_dis = curr_e_dis;

        linear_velocity = kp * curr_e_dis + kd * ddis;
        linear_velocity = max(0.1, min(linear_velocity, 0.6));

        if (center_depth < 2.1) {
            stop_robot();
            ROS_INFO("too close");
            close_person = true;
        }
        target_position.x = curr_position[0];
        target_position.y = curr_position[1];
        latest_center_depth = center_depth;
    }
    //100hz주기로 예측 위치 계산
    void update_target_position(const ros::TimerEvent& event) {
        curr_pos[0] += dx * 0.05;
        curr_pos[1] += dy * 0.05;
        target_position.x = curr_pos[0];
        target_position.y = curr_pos[1];
        //target_pub.publish(target_position);
    }
    //지형 점수 메시지 구독
    void score_callback(const unitree_lidar_ros::PathScore::ConstPtr& msg) {
        if (msg->score.size() == 7) {
            for (int i = 0; i < 7; i++) {
                external_scores[i] = msg->score[i];
            }
            choose_best_path();
        } else {
            ROS_WARN("Received external scores have incorrect size.");
        }
    }

    void move_robot(double angular_velocity) {
        geometry_msgs::Twist move_msg;
        move_msg.linear.x = linear_velocity;
        move_msg.angular.z = angular_velocity;
        if ( !emerg_flag){
        cmd_vel_pub.publish(move_msg);
        ROS_INFO("cmd_vel: (%.2f, %.2f)", move_msg.linear.x, move_msg.angular.z);
        }
        
    }
    //후보 경로의 end_point 계산
    void calculate_end_point(double angular_velocity, double path_time, double& end_x, double& end_y) {
        double distance_travelled = 1 * path_time;
        double angle_travelled = angular_velocity * path_time;
        end_x = robot_position[0] + distance_travelled * cos(robot_heading + angle_travelled);
        end_y = robot_position[1] + distance_travelled * sin(robot_heading + angle_travelled);
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "path_selector_node");

    PathSelector ps;

    ros::Rate rate(100); 

    while (ros::ok()) {
        ros::spinOnce();


        ros::Duration time_since_last_callback = ros::Time::now() - ps.getLastCallbackTime();
        if (time_since_last_callback.toSec() > 7.0) {
            ps.stop_robot();
            ROS_WARN("Person not detected for 5 seconds. Robot stopped.");
        }


        rate.sleep();
    }

    return 0;
}
