#include "covariance_markers_alg_node.h"

CovarianceMarkersAlgNode::CovarianceMarkersAlgNode(void)
{
  //init class attributes if necessary
  this->loop_rate_ = 1;//in [Hz]

  // [init publishers]
  this->markers_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("markers", 100);

  // [init subscribers]
  this->odom_subscriber_ = this->public_node_handle_.subscribe("odom", 100, &CovarianceMarkersAlgNode::odom_callback, this);
  new_marker_=false;

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

CovarianceMarkersAlgNode::~CovarianceMarkersAlgNode(void)
{
  // [free dynamic memory]
}

void CovarianceMarkersAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //this->MarkerArray_msg.data = my_var;

  // [fill srv structure and make request to the server]
  MarkerArray_msg_ = alg_.marker_array();

  // [fill action structure and make request to the action server]

  // [publish messages]
  if(new_marker_)
  {
    this->markers_publisher_.publish(this->MarkerArray_msg_);
    new_marker_=false;
  }
}

/*  [subscriber callbacks] */
void CovarianceMarkersAlgNode::odom_callback(const nav_msgs::Odometry & msg)
{

  //use appropiate mutex to shared variables if necessary
  this->alg_.lock();
  //this->odom_mutex_.enter();

  //std::cout << msg->data << std::endl;
  alg_.odom_ = msg;
  ROS_INFO("CALLBACK x: %f y: %f",alg_.odom_.pose.pose.position.x,alg_.odom_.pose.pose.position.y);
  if(!new_marker_)
  {
    alg_.drawCovariance();
    new_marker_=true;
  }

  //unlock previously blocked shared variables
  this->alg_.unlock();
  //this->odom_mutex_.exit();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void CovarianceMarkersAlgNode::node_config_update(Config &config, uint32_t level)
{
}

void CovarianceMarkersAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<CovarianceMarkersAlgNode>(argc, argv, "covariance_markers_alg_node");
}
