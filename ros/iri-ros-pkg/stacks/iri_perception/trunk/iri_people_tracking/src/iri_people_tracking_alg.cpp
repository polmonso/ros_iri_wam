#include "iri_people_tracking_alg.h"

IriPeopleTrackingAlgorithm::IriPeopleTrackingAlgorithm()
{
}

IriPeopleTrackingAlgorithm::~IriPeopleTrackingAlgorithm()
{
}

void IriPeopleTrackingAlgorithm::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// IriPeopleTrackingAlgorithm Public API