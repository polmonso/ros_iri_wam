#include "filter_table_alg_node.h"

FilterTableAlgNode::FilterTableAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<FilterTableAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
    this->filtered_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("points_filtered", 1); 
    
  // [init subscribers]
    this->cloud_subscriber_ = this->public_node_handle_.subscribe("points", 1, &FilterTableAlgNode::points_callback, this);

  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

FilterTableAlgNode::~FilterTableAlgNode(void)
{
  // [free dynamic memory]
}

void FilterTableAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void FilterTableAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void FilterTableAlgNode::addNodeDiagnostics(void)
{
}

void FilterTableAlgNode::points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_INFO("Received pointcloud in subscriber");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud=this->alg_.filter_table(msg);
  sensor_msgs::PointCloud2 out;
  std::cout<<cloud->points.size()<<" "<<cloud->width<<" "<<cloud->height<<std::endl;
  pcl::toROSMsg(*cloud, out);
  this->filtered_publisher_.publish(out);
}


/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<FilterTableAlgNode>(argc, argv, "filter_table_alg_node");
}
