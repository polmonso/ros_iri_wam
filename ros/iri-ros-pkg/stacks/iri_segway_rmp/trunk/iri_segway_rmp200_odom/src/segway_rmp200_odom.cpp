#include "segway_rmp200_odom.h"

SegwayRmp200Odom::SegwayRmp200Odom()
{
  accum_th_ = 0;
}

SegwayRmp200Odom::~SegwayRmp200Odom()
{
}

void SegwayRmp200Odom::config_update(const Config& new_cfg, uint32_t level)
{
  this->config_ = new_cfg;
}

void SegwayRmp200Odom::computeOdometry(const iri_segway_rmp_msgs::SegwayRMP200Status & msg)
{
  //update current time
  current_time_ = ros::Time::now();

  //compute differencial
  double dt = (current_time_ - last_time_).toSec();

  //robot frame
  //  x:fwd, y:left-side, z:up

  //update half theta
  double vth = -msg.yaw_rate;//left turn -> positive. in rads
  accum_th_ += vth*dt/2;
  
  //get current translational velocity and component velocities
  double vT  = (msg.left_wheel_velocity + msg.right_wheel_velocity)/2;
  double vx  = vT*cos(accum_th_);
  double vy  = vT*sin(accum_th_);

  //update position
  double delta_x  = vx*dt;
  double delta_y  = vy*dt;

  //update second half theta
  accum_th_ += vth*dt/2;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(accum_th_);

  //update transform message
  transform_.translation.x += delta_x;
  transform_.translation.y += delta_y;
  transform_.translation.z  = 0.0;
  transform_.rotation       = odom_quat;

  //update pose
  pose_.pose.position.x += delta_x;
  pose_.pose.position.y += delta_y;
  pose_.pose.position.z  = 0.0;
  pose_.pose.orientation = odom_quat;

  //update twist
  twist_.twist.linear.x  = vT;
  twist_.twist.linear.y  = 0.0;
  twist_.twist.linear.z  = 0.0;
  twist_.twist.angular.x = 0.0;
  twist_.twist.angular.y = 0.0;
  twist_.twist.angular.z = vth;
  
  //update last time
  last_time_ = current_time_;
}
   
int main(int argc, char *argv[])
{
  return algorithm_base::main< GenericOdometry<SegwayRmp200Odom, iri_segway_rmp_msgs::SegwayRMP200Status> >(argc, argv, "segway_rmp200_odom");
}
