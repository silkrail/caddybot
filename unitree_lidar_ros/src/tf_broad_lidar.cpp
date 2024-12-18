#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "caddy_tf");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(
          tf::Quaternion( 0, 0.685183, 0, 0.728371 ),
          tf::Vector3(0.0, 0.0, 0.45)),
          ros::Time::now(),
          "base_footprint",
          "unilidar_lidar"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(
          tf::Quaternion(0, 0, 0, 1),
          tf::Vector3(-0.007698, -0.014655, 0.00667)),
          ros::Time::now(),
          "unilidar_lidar",
          "unilidar_imu"));


    r.sleep();
  }
}
