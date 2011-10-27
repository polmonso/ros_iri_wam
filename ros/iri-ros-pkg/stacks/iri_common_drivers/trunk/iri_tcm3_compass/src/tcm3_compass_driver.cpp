#include "tcm3_compass_driver.h"


Tcm3CompassDriver::Tcm3CompassDriver()
{
  //setDriverId(driver string id);
  tcm3_ = new CTCM3();
  base_heading_ = 0;
  last_heading_ = 0;
}

bool Tcm3CompassDriver::openDriver(void)
{
  //setDriverId(driver string id);
  tcm3_->init();
  ROS_INFO("TCM3 Initiated");

  return true;
}

bool Tcm3CompassDriver::closeDriver(void)
{
  tcm3_->close();
  return true;
}

bool Tcm3CompassDriver::startDriver(void)
{
  tcm3_->addDataComponentId(kHEADING);
  tcm3_->addDataComponentId(kPANGLE);
  tcm3_->addDataComponentId(kRANGLE);
  tcm3_->addDataComponentId(kDISTORTION);
  tcm3_->setAcquisitionParameters(PUSHMODE);//1/config_.frequency);
  tcm3_->setMountingRef(config_.mounting_position);
  ROS_INFO("TCM3 Configured | mounting: %d",config_.mounting_position);
  ROS_INFO("TCM3 Starting Continuous Acquisition");
  tcm3_->startContinuousAcquisition();
  if(config_.relative)
  {
    tcm3_->getData();
    base_heading_ = (double)tcm3_->getHeading();
    ROS_DEBUG("TCM3 new base heading: %f",base_heading_);
  }
  return true;
}

bool Tcm3CompassDriver::stopDriver(void)
{
  ROS_INFO("TCM3 Stopping Continuous Acquisition");
  tcm3_->stopContinuousAcquisition();
  return true;
}

void Tcm3CompassDriver::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  // depending on current state
  // update driver with new_cfg data
  switch(this->getState())
  {
    case Tcm3CompassDriver::CLOSED:
      break;

    case Tcm3CompassDriver::OPENED:
      break;

    case Tcm3CompassDriver::RUNNING:
      if(new_cfg.relative != config_.relative)
      {
        if(new_cfg.relative)
          base_heading_ = last_heading_;
        else
          base_heading_ = 0;
        ROS_DEBUG("TCM3 new base heading: %f",base_heading_);
      }
      break;
  }

  // save the current configuration
  config_=new_cfg;

  this->unlock();
}

Tcm3CompassDriver::~Tcm3CompassDriver()
{
  delete tcm3_;
}

// -------------------------------------------------------------------------- //

void Tcm3CompassDriver::getData(double & heading, double & pitch, double & roll, bool & distortion)
{
  tcm3_->getData();

  last_heading_ = (double)tcm3_->getHeading();
  heading       = last_heading_ - base_heading_;
  pitch         = (double)tcm3_->getPitch();
  roll          = (double)tcm3_->getRoll();
  distortion    = (bool)tcm3_->getDistortion();

  ROS_DEBUG("heading: %f (%f) pitch: %f roll: %f",heading,last_heading_,pitch,roll);
}


