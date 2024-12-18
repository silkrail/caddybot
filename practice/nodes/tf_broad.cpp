#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "caddy_tf");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(
          tf::Quaternion(0.0299955, 0, 0.99955, 0),
          tf::Vector3(0.0, 0.0, 0.3)),
          ros::Time::now(),
          "base_footprint",
          "base_link"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(
          tf::Quaternion(0, 0, 1, 0),
          tf::Vector3(-0.1, 0.0, 0.0)),
          ros::Time::now(),
          "base_link",
          "imu_link"));

    r.sleep();
  }
}
