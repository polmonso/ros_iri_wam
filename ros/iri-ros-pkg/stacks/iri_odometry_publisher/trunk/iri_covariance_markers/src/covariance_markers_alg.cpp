#include "covariance_markers_alg.h"

CovarianceMarkersAlgorithm::CovarianceMarkersAlgorithm()
{
  counter_ = 0;
}

CovarianceMarkersAlgorithm::~CovarianceMarkersAlgorithm()
{
}

void CovarianceMarkersAlgorithm::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

// CovarianceMarkersAlgorithm Public API
void CovarianceMarkersAlgorithm::drawCovariance()
{
  current_marker_.header.stamp = ros::Time::now();
  current_marker_.header.frame_id = "/teo/odom";
  current_marker_.header.seq = counter_++;
  current_marker_.ns = "/teo";
  current_marker_.id = counter_;
  current_marker_.action = visualization_msgs::Marker::ADD;

  current_marker_.pose.position.x = odom_.pose.pose.position.x;
  current_marker_.pose.position.y = odom_.pose.pose.position.y;

  ROS_DEBUG("ALG x: %f y: %f",odom_.pose.pose.position.x,odom_.pose.pose.position.y);

  // transformar de Eigen::Matrix2f a Float32[9]
  Eigen::Matrix2f covs;
  covs(0)=odom_.pose.covariance[0];
  covs(1)=odom_.pose.covariance[1];
  covs(2)=odom_.pose.covariance[3];
  covs(3)=odom_.pose.covariance[4];

  ROS_DEBUG("cov: %f %f %f %f",covs(0),covs(1),covs(2),covs(3));

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(covs);

  const Eigen::Vector2f& eigValues (eig.eigenvalues());
  const Eigen::Matrix2f& eigVectors (eig.eigenvectors());

  float angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));

  current_marker_.type = visualization_msgs::Marker::CYLINDER;

  double lengthMajor = sqrt(eigValues[0]);
  double lengthMinor = sqrt(eigValues[1]);

  ROS_DEBUG("lengthMajor/Minor: %f %f",lengthMajor,lengthMinor);

  current_marker_.scale.x = lengthMajor+0.5;
  current_marker_.scale.y = lengthMinor+0.5;
  current_marker_.scale.z = 0.01;

  current_marker_.color.a = 0.5;
  current_marker_.color.r = 0.45;
  current_marker_.color.g = 0.95;
  current_marker_.color.b = 0.65;


  current_marker_.pose.orientation.w = cos(angle*0.5);
  current_marker_.pose.orientation.z = sin(angle*0.5);

  marker_array_.markers.push_back(current_marker_);
}

visualization_msgs::MarkerArray CovarianceMarkersAlgorithm::marker_array()
{
  return marker_array_;
}