#include "wam_dmp_tracker_driver_node.h"

WamDmpTrackerDriverNode::WamDmpTrackerDriverNode(ros::NodeHandle &nh) : 
  iri_base_driver::IriBaseNodeDriver<WamDmpTrackerDriver>(nh)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

void WamDmpTrackerDriverNode::mainNodeThread(void)
{
  //lock access to driver if necessary
  this->driver_.lock();

  // [fill msg Header if necessary]
  //<publisher_name>.header.stamp = ros::Time::now();
  //<publisher_name>.header.frame_id = "<publisher_topic_name>";

  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]

  //unlock access to driver if previously blocked
  this->driver_.unlock();
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void WamDmpTrackerDriverNode::postNodeOpenHook(void)
{
}

void WamDmpTrackerDriverNode::addNodeDiagnostics(void)
{
}

void WamDmpTrackerDriverNode::addNodeOpenedTests(void)
{
}

void WamDmpTrackerDriverNode::addNodeStoppedTests(void)
{
}

void WamDmpTrackerDriverNode::addNodeRunningTests(void)
{
}

void WamDmpTrackerDriverNode::reconfigureNodeHook(int level)
{
}

WamDmpTrackerDriverNode::~WamDmpTrackerDriverNode(void)
{
  // [free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<WamDmpTrackerDriverNode>(argc, argv, "wam_dmp_tracker_driver_node");
}
