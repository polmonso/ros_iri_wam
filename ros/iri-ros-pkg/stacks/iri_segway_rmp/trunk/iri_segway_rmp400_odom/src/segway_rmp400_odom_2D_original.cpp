#include "segway_rmp400_odom_2D.h"

SegwayRmp400Odom::SegwayRmp400Odom() :
  accum_th_(0.0)
{ }

SegwayRmp400Odom::~SegwayRmp400Odom()
{ }

void 
SegwayRmp400Odom::config_update(const Config& new_cfg, uint32_t level)
{
  config_ = new_cfg;
}

//robot frame
  //  x:fwd, y:left-side, z:up
void
SegwayRmp400Odom::computeOdometry(const iri_segway_rmp_msgs::SegwayRMP400Status & msg)
{
  current_time_ = ros::Time::now();
  // calculate the delta T
  double dt = (current_time_ - last_time_).toSec();

  //left turn -> positive. in rads
  double vth = - ((msg.rmp200[0].yaw_rate + msg.rmp200[1].yaw_rate) / 2);
  // First, we will only use the half of time. See hearder file for info.
  accum_th_ += vth * dt/2;

  //get current translational velocity and component velocities
  double v_left_wheels  = (msg.rmp200[0].left_wheel_velocity +
                           msg.rmp200[1].left_wheel_velocity) / 2;
  double v_right_wheels = (msg.rmp200[0].right_wheel_velocity +
                           msg.rmp200[1].right_wheel_velocity) / 2;

  double vT  = (v_left_wheels + v_right_wheels) / 2;
  double vx  = vT * cos(accum_th_);
  double vy  = vT * sin(accum_th_);
  // Integrate velocities with time to get current position
  double xx  = vx * dt;
  double yy  = vy * dt;

  //update second half theta
  accum_th_ += vth * dt/2;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(accum_th_);

  transform_.translation.x += xx;
  transform_.translation.y += yy;
  transform_.translation.z  = 0.0;
  transform_.rotation       = odom_quat;

  pose_.position.x += xx;
  pose_.position.y += yy;
  pose_.position.z  = 0.0;
  pose_.orientation = odom_quat;

  twist_.linear.x  = vx;
  twist_.linear.y  = 0.0;
  twist_.linear.z  = 0.0;
  twist_.angular.x = 0.0;
  twist_.angular.y = 0.0;
  twist_.angular.z = vth;

  //update last time
  last_time_ = current_time_;
}

int main(int argc, char *argv[])
{
  return algorithm_base::main< GenericOdometry<SegwayRmp400Odom, iri_segway_rmp_msgs::SegwayRMP400Status> >(argc, argv, "segway_rmp400_odom");
}
