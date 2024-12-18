// 받은 속도값으로 모터 제어+엔코더 값을 읽어 퍼블리시하는 node
#include <ros.h>
#include <Wire.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>




const double radius = 0.040;  //바퀴 반지름[m] 우리껀0.064
const double width = 0.520;  //두 바퀴 사이 거리[m]
double linear_speed_cmd = 0; //AGV 선형 속도[m/s]
double angular_speed_cmd = 0; //AGV 각속도[rad/s]
double speed_cmd_left = 0;  //왼쪽 바퀴 속도[rpm]
double speed_cmd_right = 0; //오른쪽 바퀴 속도[rpm]

void publlish_encoder();


ros::NodeHandle  nh;

// cmd_vel 콜백 함수
void AGVcontrol_cmd (const geometry_msgs::Twist& cmd_vel){


  // cmd_vel에서 속도 추출
  linear_speed_cmd = cmd_vel.linear.x;
  angular_speed_cmd = cmd_vel.angular.z;
  
  speed_cmd_left = (linear_speed_cmd + (angular_speed_cmd * width )) * 60/(2*3.14)/radius;
  speed_cmd_right = (linear_speed_cmd - (angular_speed_cmd * width )) * 60/(2*3.14)/radius;

  // 모터제어기 명령 command
  String mvc_cmd = "mvc=";
  String mvc_cmd2 = "mvc=";
  String comma = ",";

  mvc_cmd += String(speed_cmd_right) + comma + String(-speed_cmd_left);
  mvc_cmd2+= String(speed_cmd_right) + comma + String(speed_cmd_left);
  Serial3.println(mvc_cmd); // 속도명령을 내리고 엔코더값을 읽어옴.
  Serial2.println(mvc_cmd2);


}

// subscriber
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", AGVcontrol_cmd);

// publisher
geometry_msgs::PointStamped wheel;
ros::Publisher enc_pub("encoder_data", &wheel);



void setup()
{
  Serial3.begin(115200);
  Serial3.println("co1=1");
  Serial3.println("co2=1");
  Serial2.begin(115200);
  Serial2.println("co1=1");
  Serial2.println("co2=1");
  nh.initNode();
  nh.subscribe(cmd_vel);
  nh.advertise(enc_pub);

  

}

void loop()
{
  
  nh.spinOnce();
  publlish_encoder();// 엔코더 값을 읽어 퍼블리시하는 추가 함수

  //delay(37);
}

void publlish_encoder(){

  String rec_encoder = "";
  String str_l_wheel, str_r_wheel;
  Serial3.print("mp");
  delay(10); ////데이터 수신 delay
  do {
      rec_encoder += (char)Serial3.read();
      //str_data.data += (char)Serial3.read();
      
    } while (Serial3.available());

  // 읽어온 엔코더값은 mvc = _____, _____ 의 형태이므로 숫자값으로 바꿔줘야함
  int find_equal = rec_encoder.indexOf('=', 0); //from 0
  int find_dot = rec_encoder.indexOf(',', 0);
  int find_positive = rec_encoder.indexOf('-', 0);
  // 엔코더 값 추출
  for (int i = find_equal + 1; i < find_dot; i++) {str_l_wheel += rec_encoder[i];}
  for (int i = find_dot + 1; i < rec_encoder.length(); i++) {str_r_wheel += rec_encoder[i];}
  // 엔코더 값을 정수로 메시지에 추가

  wheel.point.x = str_l_wheel.toInt()*1.0; // 왼쪽 엔코더 값
  wheel.point.y = -(str_r_wheel.toInt()*1.0); // 오른쪽 엔코더 값
  //wheel.point.z = 0.0;
  
  wheel.header.stamp = nh.now();
  
  enc_pub.publish(&wheel);
  Serial3.print("\n");
  
}
