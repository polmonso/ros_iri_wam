#include "zyonz_tof_color_driver.h"

ZyonzTofColorDriver::ZyonzTofColorDriver()
{
  //setDriverId(driver string id);
}

bool ZyonzTofColorDriver::openDriver(void)
{
  //setDriverId(driver string id);

  return true;
}

bool ZyonzTofColorDriver::closeDriver(void)
{
  return true;
}

bool ZyonzTofColorDriver::startDriver(void)
{
  return true;
}

bool ZyonzTofColorDriver::stopDriver(void)
{
  return true;
}

void ZyonzTofColorDriver::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();
  
  // depending on current state
  // update driver with new_cfg data
  switch(this->getState())
  {
    case ZyonzTofColorDriver::CLOSED:
      break;

    case ZyonzTofColorDriver::OPENED:
      break;

    case ZyonzTofColorDriver::RUNNING:
      break;
  }

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

ZyonzTofColorDriver::~ZyonzTofColorDriver()
{
}
