#include "asterx1_driver_node.h"

Asterx1DriverNode::Asterx1DriverNode(ros::NodeHandle &nh) : iri_base_driver::IriBaseNodeDriver<Asterx1Driver>(nh)
{
	
  //string for port names
  std::string port_name;
	
  //init class attributes if necessary
  this->loop_rate_ = 2;//in [Hz]. Driver does not necessarily iterates at the same rate
  
  // [init publishers]
  port_name = ros::names::append(ros::this_node::getName(), "gpsStandard"); 
  this->gpsStandard = this->public_node_handle_.advertise<sensor_msgs::NavSatFix>(port_name, 100);

  //port_name = ros::names::append(ros::this_node::getName(), "gpsExtended"); 
  //this->gpsExtended = this->public_node_handle_.advertise<iri_common_msgs::gpsExtended>(port_name, 100);
 
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

void Asterx1DriverNode::mainNodeThread(void)
{
	
  ros::Time driverTS;
  
  //lock access to driver if necessary
  this->driver_.lock();

  //read device data. Blocks here (with timeout) if no data avaiolable.
  driver_.readDataFromDevice();
  
  // [fill msg Header if necessary]
  driverTS.fromSec( driver_.getTimeStamp() ); //time stamp provided by the driver process
  gpsStandardMsg.header.stamp = driverTS; //ros::Time::now();
  gpsStandardMsg.header.frame_id = "gpsStandard";
  
  // [fill msg structures]
  if (driver_.getStatus() == ALL_OK)
  {
	  gpsStandardMsg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  }
  else
  {
	  gpsStandardMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
  }
  gpsStandardMsg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  
  //geodetic position
  gpsStandardMsg.latitude = driver_.getLat(inDEGREES);
  gpsStandardMsg.longitude = driver_.getLon(inDEGREES);
  gpsStandardMsg.altitude = driver_.getAlt();
  
  //fill ENU covariance matrix row by row 
  gpsStandardMsg.position_covariance[0] = driver_.getCxxEnu(); 
  gpsStandardMsg.position_covariance[1] = driver_.getCxyEnu();
  gpsStandardMsg.position_covariance[2] = driver_.getCxzEnu(); 
  gpsStandardMsg.position_covariance[3] = driver_.getCxyEnu(); //cyx = cxy
  gpsStandardMsg.position_covariance[4] = driver_.getCyyEnu();
  gpsStandardMsg.position_covariance[5] = driver_.getCyzEnu();
  gpsStandardMsg.position_covariance[6] = driver_.getCxzEnu(); //czx = cxz
  gpsStandardMsg.position_covariance[7] = driver_.getCyzEnu(); //czy = cyz
  gpsStandardMsg.position_covariance[8] = driver_.getCzzEnu();
  gpsStandardMsg.position_covariance_type = gpsStandardMsg.COVARIANCE_TYPE_KNOWN; //to do ....
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  gpsStandard.publish(gpsStandardMsg);

  //unlock access to driver if previously blocked
  this->driver_.unlock();
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void Asterx1DriverNode::postNodeOpenHook(void)
{
}

void Asterx1DriverNode::addNodeDiagnostics(void)
{
}

void Asterx1DriverNode::addNodeOpenedTests(void)
{
}

void Asterx1DriverNode::addNodeStoppedTests(void)
{
}

void Asterx1DriverNode::addNodeRunningTests(void)
{
}

void Asterx1DriverNode::reconfigureNodeHook(int level)
{
}

Asterx1DriverNode::~Asterx1DriverNode()
{
  //[free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<Asterx1DriverNode>(argc,argv,"asterx1_driver_node");
}
