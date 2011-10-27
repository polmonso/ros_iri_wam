#include "hokuyo_laser_driver.h"
#include <cmath>
#include "ctime.h"

HokuyoLaserDriver::HokuyoLaserDriver()
{
  //setDriverId(driver string id);

  this->laser = NULL;
  std::string laser_id(ros::this_node::getName());
  replace(laser_id.begin(), laser_id.end(), '/', '_');
  ROS_INFO("Laser ID: %s", laser_id.c_str());

  try
  {
    this->laser = new CHokuyo(laser_id);
    setDriverId(this->laser->get_id());
  }
  catch (CException & e)
  {
    ROS_FATAL("'%s'",e.what().c_str());
  }

  this->ip = "192.168.0.10";
  this->port = "/dev/ttyACM0";
  this->use_eth = true;
  this->number_of_scans = 0;

}

bool HokuyoLaserDriver::openDriver(void)
{
  //setDriverId(driver string id);
  try
  {
    ROS_DEBUG("eth:%d ip:%s usb:%s",this->use_eth,this->ip.c_str(),this->port.c_str());
    if(this->config_.use_ethernet)
    {
      setIP(this->config_.ip_address);
      this->laser->connect(this->eth_cfg);
    }else
    {
      setPort(this->config_.usb_port);
      this->laser->connect(this->usb_cfg);
    }

    // get specs
    this->laser->get_specifications(this->specs);
    // treatment
    this->range_min = this->specs.min_dist/1000; // [m]
    this->range_max = this->specs.max_dist/1000; // [m]
    this->angle_increment = 2*M_PI / this->specs.num_steps; // [rad]
    this->angle_min = (this->specs.first_step - this->specs.center_step)  * this->angle_increment; // [rad]
    this->angle_max = (this->specs.last_step  - this->specs.center_step)  * this->angle_increment; // [rad]
    this->scan_time = 0.025;
    this->time_increment = 0.0001;
    // config scan

  }catch( CHokuyoException &e )
  {
    ROS_FATAL("'%s'",e.what().c_str());
  }

  return true;
}

bool HokuyoLaserDriver::closeDriver(void)
{
  try
  {
    this->laser->disconnect();
  }
  catch (CException & e)
  {
    ROS_ERROR("'%s'", e.what().c_str());
    return false;
  }
  return true;
}

bool HokuyoLaserDriver::startDriver(void)
{
  try{
    this->laser->init();
    THokuyo_scan_config sc;
    sc = this->translateConfig(this->config_);
    this->laser->config_scan(sc);
    this->laser->start(this->number_of_scans);
    ROS_INFO("Acquisition started: angle min: %f angle max: %f cluster: %d interval: %d type: %d",
             config_.angle_min, config_.angle_max, config_.cluster, config_.interval,
             config_.type);
  }
  catch (CException & e)
  {
    ROS_ERROR("'%s'", e.what().c_str());
    return false;
  }


  return true;
}

bool HokuyoLaserDriver::stopDriver(void)
{
  try{
    this->lock();
    this->laser->stop();
    this->unlock();
    ROS_DEBUG("Acquisition finished");
  }
  catch (CException & e)
  {
    ROS_ERROR("'%s'", e.what().c_str());
    return false;
  }
  return true;
}

void HokuyoLaserDriver::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  //update driver with new_cfg data

  switch(this->getState()){
    case HokuyoLaserDriver::OPENED:
      break;
    case HokuyoLaserDriver::RUNNING:
      this->frame_id = new_cfg.frame_id;
      break;
    case HokuyoLaserDriver::CLOSED:
      this->ip = new_cfg.ip_address;
      this->port = new_cfg.usb_port;
      this->use_eth = new_cfg.use_ethernet;
      break;
  }

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

HokuyoLaserDriver::~HokuyoLaserDriver()
{
  delete this->laser;
}


//                           DRIVER FUNCTIONS

///////////////////////////////////////////////////////////////////////////////
void HokuyoLaserDriver::setPort(const std::string & port)
{
  this->usb_cfg = this->laser->get_usb_config();
  if(!port.empty()) this->usb_cfg.device = port;
}

///////////////////////////////////////////////////////////////////////////////
void HokuyoLaserDriver::setIP(const std::string & ip)
{
  this->eth_cfg = this->laser->get_ethernet_config();
  this->eth_cfg.IP = ip;
}

///////////////////////////////////////////////////////////////////////////////
std::string HokuyoLaserDriver::get_new_scan_available_event()
{
  return this->laser->get_new_scan_available_event();
}

///////////////////////////////////////////////////////////////////////////////
void HokuyoLaserDriver::getScan(THokuyo_scan & scan)
{
  this->laser->get_scan(scan);
}

char HokuyoLaserDriver::setType(const int & type_num)
{
  switch(type_num)
  {
    case 1:
    default:
      return 'D';
      break;
    case 2:
      return 'E';
      break;
    case 3:
      return 'S';
      break;
  }
}

THokuyo_scan_config HokuyoLaserDriver::translateConfig(Config & cfg)
{
  THokuyo_scan_config sc;

  // -- Angles --
  sc.start_step = this->specs.center_step + (int)(cfg.angle_min/this->angle_increment);
  // limits check
  if(sc.start_step < this->specs.first_step)
    sc.start_step = this->specs.first_step;
  else if(sc.start_step > this->specs.center_step)
    sc.end_step = this->specs.center_step;
  // update config
  cfg.angle_min = (sc.start_step - this->specs.center_step) * this->angle_increment; // [rad]

  sc.end_step   = (int)( cfg.angle_max/this->angle_increment + this->specs.center_step);
  // limits check
  if(sc.end_step > this->specs.last_step)
    sc.end_step = this->specs.last_step;
  else if(sc.end_step < this->specs.center_step)
    sc.end_step = this->specs.center_step;
  // update config
  cfg.angle_max = (sc.end_step - this->specs.center_step) * this->angle_increment; // [rad]

  // -- Scan --
  sc.cluster = cfg.cluster;
  sc.interval = cfg.interval;
  sc.type = setType(cfg.type);
  sc.high_sensivity = cfg.high_sensivity;

  ROS_INFO("updated config: %d(%f) %d(%f) %d %d %c(%d)",
           sc.start_step,
           cfg.angle_min,
           sc.end_step,
           cfg.angle_max,
           sc.cluster,
           sc.interval,
           sc.type,
           cfg.type);

  return sc;
}
