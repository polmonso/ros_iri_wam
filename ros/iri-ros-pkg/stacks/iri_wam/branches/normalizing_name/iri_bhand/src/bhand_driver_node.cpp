#include "bhand_driver_node.h"

BhandDriverNode::BhandDriverNode(ros::NodeHandle &nh) : iri_base_driver::IriBaseNodeDriver<BhandDriver>(nh)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  this->bhand_cmd_server_ = this->public_node_handle_.advertiseService("bhand_cmd", &BhandDriverNode::bhand_cmdCallback, this);
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

void BhandDriverNode::mainNodeThread(void)
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
bool BhandDriverNode::bhand_cmdCallback(iri_wam_common_msgs::bhand_cmd::Request &req, iri_wam_common_msgs::bhand_cmd::Response &res) 
{ 
  bool result;
  //lock access to driver if necessary 
  this->driver_.lock(); 

  if(this->driver_.isRunning()) 
  { 
    //do operations with req and output on res 
    //res.data2 = req.data1 + my_var; 
        this->driver_.rawCommand(req.bhandcmd);
        result = true;
  } 
  else 
  { 
    std::cout << "ERROR: Driver is not on run mode yet." << std::endl; 
  } 

  //unlock driver if previously blocked 
  this->driver_.unlock(); 
  
  return result; 
}

/*  [action callbacks] */

/*  [action requests] */

void BhandDriverNode::postNodeOpenHook(void)
{
}

void BhandDriverNode::addNodeDiagnostics(void)
{
}

void BhandDriverNode::addNodeOpenedTests(void)
{
}

void BhandDriverNode::addNodeStoppedTests(void)
{
}

void BhandDriverNode::addNodeRunningTests(void)
{
}

void BhandDriverNode::reconfigureNodeHook(int level)
{
}

BhandDriverNode::~BhandDriverNode()
{
  // [free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<BhandDriverNode>(argc,argv,"bhand_driver_node");
}
