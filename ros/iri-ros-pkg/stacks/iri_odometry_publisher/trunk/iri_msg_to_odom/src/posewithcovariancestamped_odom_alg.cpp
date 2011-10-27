#include "posewithcovariancestamped_odom_alg.h"

PosewithcovariancestampedOdomAlgorithm::PosewithcovariancestampedOdomAlgorithm(void)
{
}

PosewithcovariancestampedOdomAlgorithm::~PosewithcovariancestampedOdomAlgorithm(void)
{
}

void PosewithcovariancestampedOdomAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// PosewithcovariancestampedOdomAlgorithm Public API