// 엔코더 메시지처럼 위장한 메시지를 쏘아주는 코드
#include <ros.h>
#include <Wire.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/time.h>
 

ros::NodeHandle  nh;

geometry_msgs::PointStamped wheel;
ros::Publisher enc_pub("encoder_data", &wheel);

//unsigned int l_encoder = 0;
//unsigned int r_encoder = 0;

void publish_encoder();

unsigned int l_encoder = 0;
unsigned int r_encoder = 0;
void setup()
{
  nh.initNode();

  nh.advertise(enc_pub);
}

void loop()
{
  publish_encoder();
  nh.spinOnce();
  delay(100);
}
void publish_encoder(){
  l_encoder+=2;
  r_encoder+=3;

  wheel.point.x = l_encoder*1.0;
  wheel.point.y = r_encoder*1.0;
  wheel.point.z = 0.0;
  wheel.header.stamp = nh.now();

  enc_pub.publish(&wheel);

  
}
