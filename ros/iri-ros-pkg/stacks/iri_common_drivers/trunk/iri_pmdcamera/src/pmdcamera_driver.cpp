#include "pmdcamera_driver.h"
//#include "exceptions.h"
#include "pmdcamera_exceptions.h"

PmdcameraDriver::PmdcameraDriver(): cameraType_(pmd_camcube::invalid)
{
  //setDriverId(driver string id);  
  this->PmdCC_ = new pmd_camcube::PmdCamcube ();
}

bool PmdcameraDriver::openDriver(void)
{
  //setDriverId(driver string id);

  return true;
}

bool PmdcameraDriver::closeDriver(void)
{
  return true;
}

bool PmdcameraDriver::startDriver(void)
{
  ROS_INFO ("Opening camera connection....");
  if ((this->PmdCC_ != NULL) && (cameraType_!=pmd_camcube::invalid))
  {
    try
    {
      this->PmdCC_->open (true, cameraType_);
      ROS_INFO ("Open camera connection");
      return true;
    }
    catch (CException & e)
    {
      ROS_ERROR ("Open camera connection FAILED!!! ");
      std::cout << e.what () << std::endl;
      return false;
    }
  }    
  return false;
}

bool PmdcameraDriver::stopDriver(void)
{
  if (this->PmdCC_ != NULL)
  {
    PmdCC_->close ();
    ROS_INFO ("Close camera connection");
  }
  return true;
}

void PmdcameraDriver::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();
  
  //update driver with new_cfg data
  if (this->getState () == 2)   //running
  {
//if(this->PmdCC_!=NULL){  
    ROS_INFO ("Reconfigure request :%d, level %d", new_cfg.integration_time, level);
    try
    {
      this->PmdCC_->set_integration_time (new_cfg.integration_time);
      this->PmdCC_->set_min_depth_limit(new_cfg.min_dist);
      this->PmdCC_->set_max_depth_limit(new_cfg.max_dist);
      this->PmdCC_->set_modulation_frequency(21000000);
    }
    catch (CException & e)
    {
      std::cout << e.what () << std::endl;
    }
  }
  // save the current configuration
  this->config_ = new_cfg;
  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

PmdcameraDriver::~PmdcameraDriver()
{  
  if (this->PmdCC_ != NULL)
  {
    delete this->PmdCC_;
    this->PmdCC_ = NULL;
  }
}

//void CCamcubeDriver::readData (sensor_msgs::PointCloud &cloud)
void
PmdcameraDriver::readData ()
{
  try {
    this->PmdCC_->readData ();    
  }
  catch (CException & e)
  {
    ROS_ERROR ("Read camera: FAILED!!! ");
    std::cout << e.what () << std::endl;
  }
}

float *
PmdcameraDriver::get3DImage ()
{
  if (this->PmdCC_ != NULL)
  {
    return (this->PmdCC_->get3DData ());
  }
  else
    return NULL;
}

float *
PmdcameraDriver::getIntensityImage ()
{
  if (this->PmdCC_ != NULL)
  {
    return (this->PmdCC_->getIntensityData ());
  }
  else
    return NULL;
}

float *
PmdcameraDriver::getAmplitudeImage ()
{
  if (this->PmdCC_ != NULL)
  {
    return (this->PmdCC_->getAmplitudeData ());
  }
  else
    return NULL;
}

float *
PmdcameraDriver::getDepthImage ()
{
  if (this->PmdCC_ != NULL)
  {
    return (this->PmdCC_->getDepthData ());
  }
  else
    return NULL;
}

int PmdcameraDriver::getWidth ()
{
  if (this->PmdCC_ != NULL)
  {
    return (this->PmdCC_->get_width ());
  }
  else
    return NULL;
}

int PmdcameraDriver::getHeight ()
{
  if (this->PmdCC_ != NULL)
  {
    return (this->PmdCC_->get_height ());
  }
  else
    return NULL;
}

void PmdcameraDriver::setCameraType(pmd_camcube::cameraType cameraType)
{
  cameraType_ = cameraType;
}
