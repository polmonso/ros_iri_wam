#include "hokuyo_laser3d_driver.h"
#include <sstream>
#include <stdlib.h>

HokuyoLaser3dDriver::HokuyoLaser3dDriver()
{
  //setDriverId(driver string id);
  std::string h3d_id(ros::this_node::getName());
  replace(h3d_id.begin(), h3d_id.end(), '/', '_');
  ROS_INFO("Laser ID: %s", h3d_id.c_str());
  this->id = h3d_id;

  try
  {
    this->h3d = new CH3D();
    setDriverId(h3d_id);
  }
  catch (CException & e)
  {
    ROS_FATAL("'%s'",e.what().c_str());
  }
}

bool HokuyoLaser3dDriver::openDriver(void)
{
  //setDriverId(driver string id);
  try{
    this->h3d->setLaserPort(this->config_.laser_port);
    this->h3d->setVerbose(true);
    this->h3d->setEndType(by_user,0);
    this->h3d->init();
  }catch (CException & e)
  {
    ROS_FATAL("'%s'",e.what().c_str());
  }
  return true;
}

bool HokuyoLaser3dDriver::closeDriver(void)
{
  try{
    this->h3d->close();
  }catch (CException & e)
  {
    ROS_FATAL("'%s'",e.what().c_str());
  }
  return true;
}

bool HokuyoLaser3dDriver::startDriver(void)
{
  try{
    this->h3d->setScanMode(this->config_.scan_mode);
    this->h3d->setLaserMode(this->config_.laser_mode);
    this->h3d->setResolution(this->config_.resolution);
    //this->h3d->setFiLimits(this->config_.fi_start,this->config_.fi_end);
    //this->h3d->setThLimits(this->config_.theeta_start,this->config_.theeta_end);
    if(this->config_.continuous)
      this->h3d->startAcquisition();
  }catch (CException & e)
  {
    ROS_FATAL("'%s'",e.what().c_str());
  }
  return true;
}

bool HokuyoLaser3dDriver::stopDriver(void)
{
  try{
    if(this->config_.continuous)
      this->h3d->stopAcquisition();
  }catch (CException & e)
  {
    ROS_FATAL("'%s'",e.what().c_str());
  }
  return true;
}

void HokuyoLaser3dDriver::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  //update driver with new_cfg data

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

HokuyoLaser3dDriver::~HokuyoLaser3dDriver()
{
  delete this->h3d;
}

// -----------------------------------------------------------------------------

bool HokuyoLaser3dDriver::get3DScan(clouddataraw & cloud){
  try{
    this->h3d->get3DScan(cloud);
  }catch (CException & e)
  {
    ROS_ERROR("'%s'",e.what().c_str());
  }
  return true;
}

bool HokuyoLaser3dDriver::cloudToXYZ(const clouddataraw & raw, clouddataxyz & xyz){
  try{
    this->h3d->cloudToXYZ(raw,xyz);
  }catch (CException & e)
  {
    ROS_ERROR("'%s'",e.what().c_str());
  }
  return true;
}

bool HokuyoLaser3dDriver::single3DScan(clouddataxyz & xyz_scan){

  if(!this->config_.continuous)
  {
    CEventServer *event_server;
    event_server = CEventServer::instance();
    std::list<std::string> events;
    events.push_back(NEW3DSCAN);       // event 0
    events.push_back(ENDACQUISITION);  // event 1
    std::stringstream filepath;

    ROS_INFO("startAcquisition");

    this->h3d->startAcquisition();

    try
    {
      clouddataraw *raw_scan;
      raw_scan = new clouddataraw;
      filepath << "/home/" << getenv("USER") << "/Experiments/" << this->id << " " << ros::Time::now() << ".txt";
      //xyz_scan = new clouddataxyz;
      if( event_server->wait_first(events) == 0 )
      {
        this->h3d->get3DScan(*raw_scan);
        this->h3d->cloudToXYZ(*raw_scan,xyz_scan);
        this->h3d->writeCloudToFile( *raw_scan, filepath.str().c_str());
      }
      delete raw_scan;
    }
    catch(CException &e)
    {
      ROS_ERROR("'%s'",e.what().c_str());
    }


    this->h3d->stopAcquisition();
  }
  return true;
}


