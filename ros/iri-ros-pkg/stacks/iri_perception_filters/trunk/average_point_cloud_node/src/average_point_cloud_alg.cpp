#include "average_point_cloud_alg.h"

AveragePointCloudAlgorithm::AveragePointCloudAlgorithm()
{
}

AveragePointCloudAlgorithm::~AveragePointCloudAlgorithm()
{
}

void AveragePointCloudAlgorithm::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// AveragePointCloudAlgorithm Public API