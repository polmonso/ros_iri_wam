#include "wam_controller_driver_node.h"

WamControllerDriverNode::WamControllerDriverNode(ros::NodeHandle &nh) : 
  iri_base_driver::IriBaseNodeDriver<WamControllerDriver>(nh)
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

void WamControllerDriverNode::mainNodeThread(void)
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

void WamControllerDriverNode::postNodeOpenHook(void)
{
}

void WamControllerDriverNode::addNodeDiagnostics(void)
{
}

void WamControllerDriverNode::addNodeOpenedTests(void)
{
}

void WamControllerDriverNode::addNodeStoppedTests(void)
{
}

void WamControllerDriverNode::addNodeRunningTests(void)
{
}

void WamControllerDriverNode::reconfigureNodeHook(int level)
{
}

WamControllerDriverNode::~WamControllerDriverNode(void)
{
  // [free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<WamControllerDriverNode>(argc, argv, "wam_controller_driver_node");
}
