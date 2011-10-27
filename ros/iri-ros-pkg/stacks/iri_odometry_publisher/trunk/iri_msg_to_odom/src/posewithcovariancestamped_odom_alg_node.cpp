#include "posewithcovariancestamped_odom_alg_node.h"

PosewithcovariancestampedOdomAlgNode::PosewithcovariancestampedOdomAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<PosewithcovariancestampedOdomAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->odom_publisher_ = this->public_node_handle_.advertise<nav_msgs::Odometry>("odom", 100);
  
  // [init subscribers]
  this->pose_subscriber_ = this->public_node_handle_.subscribe("pose", 100, &PosewithcovariancestampedOdomAlgNode::pose_callback, this);

  this->public_node_handle_.param<std::string>("child_id", child_id_, "base_footprint");
 
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

PosewithcovariancestampedOdomAlgNode::~PosewithcovariancestampedOdomAlgNode(void)
{
  // [free dynamic memory]
}

void PosewithcovariancestampedOdomAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //this->Odometry_msg.data = my_var;
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  
}

/*  [subscriber callbacks] */
void PosewithcovariancestampedOdomAlgNode::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) 
{ 
  //ROS_DEBUG("PosewithcovariancestampedOdomAlgNode::pose_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary 
  //this->alg_.lock(); 
  //this->pose_mutex_.enter(); 

  //std::cout << msg->data << std::endl;
  this->Odometry_msg_.header = msg->header;
  this->Odometry_msg_.child_frame_id = child_id_;
  this->Odometry_msg_.pose   = msg->pose;

  this->odom_publisher_.publish(this->Odometry_msg_);

  //unlock previously blocked shared variables 
  //this->alg_.unlock(); 
  //this->pose_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void PosewithcovariancestampedOdomAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void PosewithcovariancestampedOdomAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<PosewithcovariancestampedOdomAlgNode>(argc, argv, "posewithcovariancestamped_odom_alg_node");
}
