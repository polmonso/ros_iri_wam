#include "image_publisher_alg.h"

ImagePublisherAlgorithm::ImagePublisherAlgorithm(void)
{
}

ImagePublisherAlgorithm::~ImagePublisherAlgorithm(void)
{
}

void ImagePublisherAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// ImagePublisherAlgorithm Public API