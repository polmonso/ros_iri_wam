#include "segway_rmp400_odom_2D.h"

SegwayRmp400Odom::SegwayRmp400Odom() :
  accum_th_(0.0)
{

  mu_v_ = 1;
  mu_th_ = 1;
  sigma_factor_v_ = 0.95;
  sigma_factor_th_ = 0.70;

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
  ROS_DEBUG("dt: %f",dt);

  //left turn -> positive. in rads
  double vth = - ((msg.rmp200[0].yaw_rate + msg.rmp200[1].yaw_rate) / 2);
  // First, we will only use the half of time. See hearder file for info.
  accum_th_ += vth * dt/2;
//  ROS_INFO("dt odo2D: %f", dt);
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

  pose_.pose.position.x += xx;
  pose_.pose.position.y += yy;
  pose_.pose.position.z  = 0.0;
  pose_.pose.orientation = odom_quat;

  pose_.covariance[0]  =
  pose_.covariance[7]  =
  pose_.covariance[14] =
  pose_.covariance[21] =
  pose_.covariance[28] =
  pose_.covariance[35] = 0.5;

  twist_.twist.linear.x  = vT;
  twist_.twist.linear.y  = 0.0;
  twist_.twist.linear.z  = 0.0;
  twist_.twist.angular.x = 0.0;
  twist_.twist.angular.y = 0.0;
  twist_.twist.angular.z = vth;

  // Add the covariances to use with EKF
  // for x and y velocity, Error ~= +-10%
  twist_.covariance[0]  = pow(0.1*vx,2);
  twist_.covariance[7]  = pow(0.1*vy,2);
  twist_.covariance[14] =
  twist_.covariance[21] =
  twist_.covariance[28] = 1;
  // for theta velocity, Error ~= +-50%
  twist_.covariance[35] = pow(0.5*vth,2);

  //update last time
  last_time_ = current_time_;
}

int main(int argc, char *argv[])
{
  return algorithm_base::main< GenericOdometry<SegwayRmp400Odom, iri_segway_rmp_msgs::SegwayRMP400Status> >(argc, argv, "segway_rmp400_odom");
}
