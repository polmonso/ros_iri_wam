#include "iri_laser_people_detection_alg.h"

IriLaserPeopleDetectionAlgorithm::IriLaserPeopleDetectionAlgorithm()
{
}

IriLaserPeopleDetectionAlgorithm::~IriLaserPeopleDetectionAlgorithm()
{
}

void IriLaserPeopleDetectionAlgorithm::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// IriLaserPeopleDetectionAlgorithm Public API