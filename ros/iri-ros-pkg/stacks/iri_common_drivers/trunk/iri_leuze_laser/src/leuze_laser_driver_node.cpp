#include "leuze_laser_driver_node.h"

LeuzeLaserDriverNode::LeuzeLaserDriverNode(ros::NodeHandle &nh) : iri_base_driver::IriBaseNodeDriver<LeuzeLaserDriver>(nh)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  scan_publisher_ = public_node_handle_.advertise<sensor_msgs::LaserScan>("scan", 100);
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]

  event_server_ = CEventServer::instance();
  events_.push_back(driver_.getNewScanEventId());
  events_.push_back(driver_.getNoLaserEventId());
}

void LeuzeLaserDriverNode::mainNodeThread(void)
{
  // [fill msg Header if necessary]
  //<publisher_name>.header.stamp = ros::Time::now();
  //<publisher_name>.header.frame_id = "<publisher_topic_name>";

  // [fill msg structures]
  //this->LaserScan_msg.data = my_var;
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]  

  // wait for new laser scan event
  int event_id = event_server_->wait_first(events_);
  
  switch(event_id)
  {
    //New Scan Event
    case 0:
      //ROS_INFO("New Laser Scan!");

      driver_.lock();
        LaserScan_msg_ = driver_.getScan();
      driver_.unlock();

      scan_publisher_.publish(LaserScan_msg_);
      break;

    //No Laser Detected
    case 1:
      ROS_ERROR("Laser NOT detected, please check either the serial port or the power supply");
      break;
  }
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void LeuzeLaserDriverNode::postNodeOpenHook(void)
{
}

void LeuzeLaserDriverNode::addNodeDiagnostics(void)
{
}

void LeuzeLaserDriverNode::addNodeOpenedTests(void)
{
}

void LeuzeLaserDriverNode::addNodeStoppedTests(void)
{
}

void LeuzeLaserDriverNode::addNodeRunningTests(void)
{
}

void LeuzeLaserDriverNode::reconfigureNodeHook(int level)
{
}

LeuzeLaserDriverNode::~LeuzeLaserDriverNode()
{
  // [free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<LeuzeLaserDriverNode>(argc, argv, "leuze_laser_driver_node");
}
