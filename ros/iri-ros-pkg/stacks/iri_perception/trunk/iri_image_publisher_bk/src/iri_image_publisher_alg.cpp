#include "iri_simple_perception_alg.h"

IriSimplePerceptionAlgorithm::IriSimplePerceptionAlgorithm()
{
}

IriSimplePerceptionAlgorithm::~IriSimplePerceptionAlgorithm()
{
}

void IriSimplePerceptionAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// IriSimplePerceptionAlgorithm Public API