#include "segway_rmp400_odom_3D.h"

SegwayRmp400Odom::SegwayRmp400Odom()
{
  accum_yaw_=0.0;
  accum_pitch_=0.0;
  accum_roll_=0.0;
}

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
  double vyaw   = - ((msg.rmp200[0].yaw_rate   + msg.rmp200[1].yaw_rate  ) / 2);
  double vpitch =   ((msg.rmp200[0].pitch_rate + msg.rmp200[1].pitch_rate) / 2);
  double vroll  =   ((msg.rmp200[0].roll_rate  + msg.rmp200[1].roll_rate ) / 2);
  // First, we will only use the half of time. See header file for info.
  accum_yaw_   += vyaw   * dt/2;
  accum_pitch_ += vpitch * dt/2;
  accum_roll_  += vroll  * dt/2;

  //get current translational velocity and component velocities
  double v_left_wheels  = (msg.rmp200[0].left_wheel_velocity +
                           msg.rmp200[1].left_wheel_velocity) / 2;
  double v_right_wheels = (msg.rmp200[0].right_wheel_velocity +
                           msg.rmp200[1].right_wheel_velocity) / 2;

  double vT  = (v_left_wheels + v_right_wheels) / 2;
  double vx  = vT * cos(accum_yaw_);
  double vy  = vT * sin(accum_yaw_);

  // Integrate velocities with time to get current position
  double xx  = vx * dt;
  double yy  = vy * dt;

  //update second half angles
  accum_yaw_   += vyaw   * dt/2;
  accum_pitch_ += vpitch * dt/2;
  accum_roll_  += vroll  * dt/2;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(accum_yaw_);

  transform_.translation.x += xx;
  transform_.translation.y += yy;
  transform_.translation.z  = 0.0;
  transform_.rotation       = odom_quat;

  pose_.pose.position.x += xx;
  pose_.pose.position.y += yy;
  pose_.pose.position.z  = 0.0;
  pose_.pose.orientation = odom_quat;

  twist_.twist.linear.x  = vx;
  twist_.twist.linear.y  = 0.0;
  twist_.twist.linear.z  = 0.0;
  twist_.twist.angular.x = vroll;
  twist_.twist.angular.y = vpitch;
  twist_.twist.angular.z = vyaw;

  //update last time
  last_time_ = current_time_;
}

int main(int argc, char *argv[])
{
  return algorithm_base::main< GenericOdometry<SegwayRmp400Odom, iri_segway_rmp_msgs::SegwayRMP400Status> >(argc, argv, "segway_rmp400_odom");
}
