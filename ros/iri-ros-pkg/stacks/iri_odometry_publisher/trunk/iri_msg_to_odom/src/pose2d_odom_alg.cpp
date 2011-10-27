#include "pose2d_odom_alg.h"

Pose2dOdomAlgorithm::Pose2dOdomAlgorithm()
{
}

Pose2dOdomAlgorithm::~Pose2dOdomAlgorithm()
{
}

void Pose2dOdomAlgorithm::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

// Pose2dOdomAlgorithm Public API


void Pose2dOdomAlgorithm::getOdometry(nav_msgs::Odometry & o, geometry_msgs::TransformStamped & t)
{

  double dT = ( new_time_ - old_time_ ).toSec();               // [s]
  double dx = ( new_pose2D_.x - old_pose2D_.x ) / dT;          // [m/s]
  double dy = ( new_pose2D_.y - old_pose2D_.y ) / dT;          // [m/s]
  double dth = ( new_pose2D_.theta - old_pose2D_.theta ) / dT; // [m/s]

  geometry_msgs::Quaternion or_q = tf::createQuaternionMsgFromYaw( new_pose2D_.theta );

  o.header.stamp    = new_time_;
  o.header.frame_id = parent_id_;
  o.child_frame_id  = frame_id_;

  o.pose.pose.position.x  = new_pose2D_.x;
  o.pose.pose.position.y  = new_pose2D_.y;
  o.pose.pose.position.z  = 0.0;
  o.pose.pose.orientation = or_q;

  o.pose.covariance[0]  =
  o.pose.covariance[7]  =
  o.pose.covariance[14] =
  o.pose.covariance[21] =
  o.pose.covariance[28] =
  o.pose.covariance[35] = 0.01;

  o.twist.twist.linear.x  =  dx;
  o.twist.twist.linear.y  =  dy;
  o.twist.twist.linear.z  =  0.0;
  o.twist.twist.angular.x =  0.0;
  o.twist.twist.angular.y =  0.0;
  o.twist.twist.angular.z =  dth;

  o.twist.covariance[0]  =
  o.twist.covariance[7]  =
  o.twist.covariance[14] =
  o.twist.covariance[21] =
  o.twist.covariance[28] =
  o.twist.covariance[35] = 0.01;

  t.header.stamp    = new_time_;
  t.header.frame_id = parent_id_;
  t.child_frame_id  = frame_id_;

  t.transform.translation.x = new_pose2D_.x;
  t.transform.translation.y = new_pose2D_.y;
  t.transform.translation.z = 0.0;
  t.transform.rotation      = or_q;

}