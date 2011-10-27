#include "fake_image_processing_alg.h"

FakeImageProcessingAlgorithm::FakeImageProcessingAlgorithm(void)
{
}

FakeImageProcessingAlgorithm::~FakeImageProcessingAlgorithm(void)
{
}

void FakeImageProcessingAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// FakeImageProcessingAlgorithm Public API