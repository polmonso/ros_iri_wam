#include "leuze_laser_driver.h"
#include "leuze_laser_exceptions.h"

LeuzeLaserDriver::LeuzeLaserDriver() :
  leuze_("leuze")
{
  setDriverId("leuze");
}

bool LeuzeLaserDriver::openDriver(void)
{
  try
  {
    this->lock();
      std::string port(config_.serial_port);
    this->unlock();  

    leuze_.open(port);
  }
  catch (CException & e)
  {
    leuze_.close();
    ROS_ERROR("'%s'", e.what().c_str());
    return false;
  }
  return true;
}

bool LeuzeLaserDriver::closeDriver(void)
{
  try
  {
    leuze_.close();
  }
  catch (CException & e)
  {
    ROS_ERROR("'%s'", e.what().c_str());
    return false;
  }
  return true;
}

bool LeuzeLaserDriver::startDriver(void)
{
  try
  {
    leuze_.start();
  }
  catch (CException & e)
  {
    ROS_ERROR("'%s'", e.what().c_str());
    return false;
  }
  return true;
}

bool LeuzeLaserDriver::stopDriver(void)
{
  try
  {
    leuze_.stop();
  }
  catch (CException & e)
  {
    ROS_ERROR("'%s'", e.what().c_str());
    return false;
  }
  return true;
}

void LeuzeLaserDriver::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();
  
  // depending on current state
  // update driver with new_cfg data
  switch(this->getState())
  {
    case LeuzeLaserDriver::CLOSED:
//       config_.serial_port = new_cfg.serial_port;
      break;

    case LeuzeLaserDriver::OPENED:
      break;

    case LeuzeLaserDriver::RUNNING:
      break;
  }

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

std::string LeuzeLaserDriver::getNewScanEventId(void) const
{
  return leuze_.get_new_scan_event_id();
}

std::string LeuzeLaserDriver::getNoLaserEventId(void) const
{
  return leuze_.get_laser_not_present_event_id();
}

sensor_msgs::LaserScan LeuzeLaserDriver::getScan(void)
{
  TLeuzeScan scan;
  leuze_.get_scan(&scan);
  return leuzeScantoLaserScan(scan);
}

sensor_msgs::LaserScan LeuzeLaserDriver::leuzeScantoLaserScan(const TLeuzeScan & leuze_scan)
{
//   std::cout << "config_.angle_offset=" << config_.angle_offset << std::endl;
//   std::cout << "config_.upsidedown="   << config_.upsidedown   << std::endl;

  sensor_msgs::LaserScan scan;

  scan.header.stamp    = ros::Time::now();
  scan.header.frame_id = config_.frame_id;

  unsigned int num_ranges = ((leuze_scan.stop-leuze_scan.start)/leuze_scan.res)+1;
  float angle_offset = config_.angle_offset * M_PI/180.f;
  int sign = 1;

  if (config_.upsidedown)
    sign = -1;

  scan.angle_min       = sign*(( 95.04 * M_PI/180.f) - angle_offset); //start angle of the scan [rad]
  scan.angle_max       = sign*((-95.04 * M_PI/180.f) + angle_offset);//end angle of the scan [rad]
  scan.angle_increment = sign*(-0.0251327);//-(scan.angle_min - scan.angle_max) / (double)(num_ranges-1);
  scan.time_increment  = 0.f;//time between measurements [seconds]
  scan.scan_time       = 0.0211;//time between scans [seconds]
  scan.range_min       = 0.f;//minimum range value [m]
  scan.range_max       = 50.f;//maximum range value [m]

  unsigned int num_discarded_ranges = abs(round(angle_offset/scan.angle_increment));
  unsigned int num_desired_ranges   = num_ranges-2*num_discarded_ranges;

  // [mm] (int) -> [m] (float)
  std::vector<float> ranges;

  for(unsigned int ii=0; ii<num_desired_ranges; ii++)
//     ranges.push_back((float)ii/10.0f);
    ranges.push_back((float)leuze_scan.data[ii+num_discarded_ranges]/1000.f);

  scan.ranges = ranges;//range data [m] (Note: values < range_min or > range_max should be discarded)

//   std::cout << "scan.angle_min       = " << scan.angle_min << std::endl;
//   std::cout << "scan.angle_max       = " << scan.angle_max << std::endl;
//   std::cout << "scan.angle_increment = " << scan.angle_increment << std::endl;
//   std::cout << "num_discarded_ranges = " << num_discarded_ranges << std::endl;
//   std::cout << "num_desired_ranges   = " << num_desired_ranges << std::endl;
//   std::cout << "ranges.size           = " << ranges.size() << std::endl;  
//   std::cout << std::endl;

  return scan;
}
  
LeuzeLaserDriver::~LeuzeLaserDriver()
{
}
