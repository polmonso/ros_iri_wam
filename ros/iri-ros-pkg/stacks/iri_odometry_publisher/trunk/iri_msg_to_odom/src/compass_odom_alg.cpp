#include "compass_odom_alg.h"

CompassOdomAlgorithm::CompassOdomAlgorithm()
{
}

CompassOdomAlgorithm::~CompassOdomAlgorithm()
{
}

void CompassOdomAlgorithm::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

// CompassOdomAlgorithm Public API

//    CONVERSION FROM COMPASS MSG TO ODOMETRY MSG
// -----------------------------------------------------------------------------

void CompassOdomAlgorithm::getOdometry(nav_msgs::Odometry & o, geometry_msgs::TransformStamped & t)
{
  // compass3axis [yaw,roll,-pitch]
  // odom [roll,pitch,yaw]

  double rc[3]; // last angle in radians
  double r[3];  // angles in radians
  double w[3];  // angular speed

  for(uint i=0;i<3;i++)
  {
    rc[i] = new_compass_.angles[i] * M_PI / 180;
    r[i] = ( new_compass_.angles[i] - old_compass_.angles[i] ) * M_PI / 180;
    w[i] = r[i] / ( new_compass_.header.stamp.toSec() - old_compass_.header.stamp.toSec() );
  }

  geometry_msgs::Quaternion or_q = tf::createQuaternionMsgFromRollPitchYaw( rc[2], -rc[1], -rc[0] );

  //   Header header
  //   uint32 seq
  //   time stamp
  //   string frame_id
  //   float64[3] angles
  //   float64[9] covariances
  //   bool[3] alarms
  // --------------------------
  //   Header header
  //    uint32 seq
  //    time stamp
  //    string frame_id
  o.header.stamp = new_compass_.header.stamp;
  o.header.frame_id = parent_id_;
  //   string child_frame_id
  o.child_frame_id  = new_compass_.header.frame_id;
  //   geometry_msgs/PoseWithCovariance pose
  //    geometry_msgs/Pose pose
  //      geometry_msgs/Point position
  //        float64 x
  //        float64 y
  //        float64 z
  //      geometry_msgs/Quaternion orientation
  //        float64 x
  //        float64 y
  //        float64 z
  //        float64 w
  //    float64[36] covariance
  o.pose.pose.position.x  = 0.0;
  o.pose.pose.position.y  = 0.0;
  o.pose.pose.position.z  = 0.0;
  o.pose.pose.orientation = or_q;
  for(uint i=0;i<36;i++)
    o.pose.covariance[i] = 999;
  o.pose.covariance[0] = new_compass_.covariances[0];
  o.pose.covariance[4] = new_compass_.covariances[4];
  o.pose.covariance[8] = 1;//new_compass_.covariances[8];
  o.twist.twist.linear.x  =  0.0;
  o.twist.twist.linear.y  =  0.0;
  o.twist.twist.linear.z  =  0.0;
  o.twist.twist.angular.x =  w[2];  // roll
  o.twist.twist.angular.y =  -w[1]; // pitch
  o.twist.twist.angular.z =  -w[0];  // yaw
  //o.twist.covariance = covs;

  t.header.stamp = new_compass_.header.stamp;
  t.header.frame_id = parent_id_;
  t.child_frame_id  = new_compass_.header.frame_id;
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation      = or_q;

}

void CompassOdomAlgorithm::getImuMessage(nav_msgs::Odometry & o, sensor_msgs::Imu & imu)
{
  imu.header                = o.header;
  imu.orientation           = o.pose.pose.orientation;
  imu.orientation_covariance[0] = o.pose.covariance[0];
  imu.orientation_covariance[4] = o.pose.covariance[4];
  imu.orientation_covariance[8] = o.pose.covariance[8];
  imu.angular_velocity.x    = o.twist.twist.angular.x;
  imu.angular_velocity.y    = o.twist.twist.angular.x;
  imu.angular_velocity.z    = o.twist.twist.angular.x;
  imu.linear_acceleration.x = 0.0;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = 0.0;
}






















