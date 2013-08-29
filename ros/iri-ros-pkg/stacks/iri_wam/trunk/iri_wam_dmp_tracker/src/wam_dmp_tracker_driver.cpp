#include "wam_dmp_tracker_driver.h"

WamDmpTrackerDriver::WamDmpTrackerDriver(void)
{
  //setDriverId(driver string id);
}

bool WamDmpTrackerDriver::openDriver(void)
{
  //setDriverId(driver string id);

  return true;
}

bool WamDmpTrackerDriver::closeDriver(void)
{
  return true;
}

bool WamDmpTrackerDriver::startDriver(void)
{
  return true;
}

bool WamDmpTrackerDriver::stopDriver(void)
{
  return true;
}

void WamDmpTrackerDriver::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();
  
  // depending on current state
  // update driver with new_cfg data
  switch(this->getState())
  {
    case WamDmpTrackerDriver::CLOSED:
      break;

    case WamDmpTrackerDriver::OPENED:
      break;

    case WamDmpTrackerDriver::RUNNING:
      break;
  }

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

WamDmpTrackerDriver::~WamDmpTrackerDriver(void)
{
}
