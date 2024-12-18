#include "lino_base.h"

float  linear_scale_positive = 1.0;
float  linear_scale_negative = 1.0;
float  angular_scale_positive = 1.0;
float  angular_scale_negative = 1.0;
float  two_pi =  6.28319;
float  dx = 0;
float dy = 0;
float dxy = 0;


LinoBase::LinoBase():
    linear_velocity_x_(0),
    linear_velocity_y_(0),
    angular_velocity_z_(0),
    last_vel_time_(0),
    vel_dt_(0),
    x_pos_(0),
    y_pos_(0),
    heading_(0)
{
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("raw_odom", 50);
    velocity_subscriber_ = nh_.subscribe("speed", 50, &LinoBase::velCallback, this);
}


void LinoBase::velCallback( const geometry_msgs::Vector3Stamped& vel)
{
    ros::Time current_time = ros::Time::now();
    linear_velocity_x_ = vel.vector.y*1.7;
    linear_velocity_y_ = vel.vector.x*1.7;

    vel_dt_ = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;
    dt = vel.vector.z;
    //dt  = vel_dt_;
    dxy = ( linear_velocity_x_ + linear_velocity_y_)/2;
    dth_odom = ((linear_velocity_y_-linear_velocity_x_)*dt)/wheelbase;

    dx = cos(dth_odom) * dxy;
    dy = sin(dth_odom) * dxy;

    x_pos_ += (cos(heading_ ) * dx - sin(heading_ ) * dy);
    y_pos_ += (sin(heading_ ) * dx + cos(heading_ ) * dy);
    heading_ += dth_odom;

    if(heading_  >= two_pi) heading_  -= two_pi;
    if(heading_  <= -two_pi) heading_  += two_pi;

    odom_quat.setRPY(0,0,heading_);

    
  
    /*
    double delta_heading = dth_odom * vel_dt_; //radians
    double delta_x = (dxy * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_; //m
    double delta_y = (dxy * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_; //m
    
    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;*/

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    /*
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos_;
    odom_trans.transform.translation.y = y_pos_;
    odom_trans.transform.translation.z = 0.0;

    //robot's heading in quaternion

    odom_trans.header.stamp = current_time;

    //publish robot's tf using odom_trans object
    //odom_broadcaster_.sendTransform(odom_trans);*/

    odom.header.stamp = current_time;
/*
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";*/

    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;

    //robot's heading in quaternion
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();



    if (vel.vector.x == 0 && vel.vector.y == 0){
      odom.pose.covariance[0] = 1e-9;
      odom.pose.covariance[7] = 1e-3;
      odom.pose.covariance[8] = 1e-9;
      odom.pose.covariance[14] = 1e6;
      odom.pose.covariance[21] = 1e6;
      odom.pose.covariance[28] = 1e6;
      odom.pose.covariance[35] = 1e-9;
      odom.twist.covariance[0] = 1e-9;
      odom.twist.covariance[7] = 1e-3;
      odom.twist.covariance[8] = 1e-9;
      odom.twist.covariance[14] = 1e6;
      odom.twist.covariance[21] = 1e6;
      odom.twist.covariance[28] = 1e6;
      odom.twist.covariance[35] = 1e-9;
    }

    else{
      odom.pose.covariance[0] = 1e-3;
      odom.pose.covariance[7] = 1e-3;
      odom.pose.covariance[8] = 0.0;
      odom.pose.covariance[14] = 1e6;
      odom.pose.covariance[21] = 1e6;
      odom.pose.covariance[28] = 1e6;
      odom.pose.covariance[35] = 1e3;
      odom.twist.covariance[0] = 1e-3;
      odom.twist.covariance[7] = 1e-3;
      odom.twist.covariance[8] = 0.0;
      odom.twist.covariance[14] = 1e6;
      odom.twist.covariance[21] = 1e6;
      odom.twist.covariance[28] = 1e6;
      odom.twist.covariance[35] = 1e3;
    }

    //linear speed from encoders
    odom.twist.twist.linear.x = dxy;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;

    //angular speed from encoders
    odom.twist.twist.angular.z = dth_odom;

    odom_publisher_.publish(odom);

}
/*
    ros::Time current_time = ros::Time::now();

    linear_velocity_x_ = vel.vector.x;
    linear_velocity_y_ = vel.vector.y;
    dxy = (linear_velocity_x_ + linear_velocity_y_)/2;
    dt = vel.vector.z;
    dth_odom = ((linear_velocity_x_ - linear_velocity_y_)*dt)/wheelbase;

    if (dth_odom > 0) dth_odom *= angular_scale_positive;
    if (dth_odom < 0) dth_odom *= angular_scale_negative;
    if (dxy > 0) dxy *= linear_scale_positive;
    if (dxy < 0) dxy *= linear_scale_negative;

    dx = cos(dth_odom) * dxy;
    dy = sin(dth_odom) * dxy;

    x_pos_ += (cos(heading_) * dx - sin(heading_) * dy);
    y_pos_ += (sin(heading_) * dx + cos(heading_) * dy);
    heading_ += dth_odom;

    if(heading_  >= two_pi) heading_  -= two_pi;
    if(heading_  <= -two_pi) heading_  += two_pi;

    odom_quat.setRPY(0,0,heading_);

    /*
    vel_dt_ = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;
    
    double delta_heading = dth_odom * vel_dt_; //radians
    double delta_x = (dxy * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_; //m
    double delta_y = (dxy * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_; //m
    
    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos_;
    odom_trans.transform.translation.y = y_pos_;
    odom_trans.transform.translation.z = 0.0;

    //robot's heading in quaternion

    odom_trans.header.stamp = current_time;
    //publish robot's tf using odom_trans object
    odom_broadcaster_.sendTransform(odom_trans);

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;

    //robot's heading in quaternion
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();

    if (vel.vector.x == 0 && vel.vector.y == 0){
      odom.pose.covariance[0] = 1e-9;
      odom.pose.covariance[7] = 1e-3;
      odom.pose.covariance[8] = 1e-9;
      odom.pose.covariance[14] = 1e6;
      odom.pose.covariance[21] = 1e6;
      odom.pose.covariance[28] = 1e6;
      odom.pose.covariance[35] = 1e-9;
      odom.twist.covariance[0] = 1e-9;
      odom.twist.covariance[7] = 1e-3;
      odom.twist.covariance[8] = 1e-9;
      odom.twist.covariance[14] = 1e6;
      odom.twist.covariance[21] = 1e6;
      odom.twist.covariance[28] = 1e6;
      odom.twist.covariance[35] = 1e-9;
    }

    else{
      odom.pose.covariance[0] = 1e-3;
      odom.pose.covariance[7] = 1e-3;
      odom.pose.covariance[8] = 0.0;
      odom.pose.covariance[14] = 1e6;
      odom.pose.covariance[21] = 1e6;
      odom.pose.covariance[28] = 1e6;
      odom.pose.covariance[35] = 1e3;
      odom.twist.covariance[0] = 1e-3;
      odom.twist.covariance[7] = 1e-3;
      odom.twist.covariance[8] = 0.0;
      odom.twist.covariance[14] = 1e6;
      odom.twist.covariance[21] = 1e6;
      odom.twist.covariance[28] = 1e6;
      odom.twist.covariance[35] = 1e3;
    }

    dxy = (dt == 0)?  0 : (linear_velocity_y_+linear_velocity_x_)/2;
    dth_odom = (dt == 0)? 0 : ((linear_velocity_x_ - linear_velocity_y_)*dt)/wheelbase;

    //linear speed from encoders
    odom.twist.twist.linear.x = dxy;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;

    //angular speed from encoders
    odom.twist.twist.angular.z = dth_odom;

    odom_publisher_.publish(odom);
}


*/

