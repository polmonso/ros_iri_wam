#include "wam_controller_driver.h"

WamControllerDriver::WamControllerDriver(void)
{
  //setDriverId(driver string id);
}

bool WamControllerDriver::openDriver(void)
{
  //setDriverId(driver string id);

  return true;
}

bool WamControllerDriver::closeDriver(void)
{
  return true;
}

bool WamControllerDriver::startDriver(void)
{
  return true;
}

bool WamControllerDriver::stopDriver(void)
{
  return true;
}

void WamControllerDriver::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();
  
  // depending on current state
  // update driver with new_cfg data
  switch(this->getState())
  {
    case WamControllerDriver::CLOSED:
      break;

    case WamControllerDriver::OPENED:
      break;

    case WamControllerDriver::RUNNING:
      break;
  }

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

WamControllerDriver::~WamControllerDriver(void)
{
}
