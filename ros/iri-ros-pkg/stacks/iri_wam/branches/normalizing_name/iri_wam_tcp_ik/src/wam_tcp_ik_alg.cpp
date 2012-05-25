#include "wam_tcp_ik_alg.h"

WamTcpIkAlgorithm::WamTcpIkAlgorithm(void)
{
}

WamTcpIkAlgorithm::~WamTcpIkAlgorithm(void)
{
}

void WamTcpIkAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// WamTcpIkAlgorithm Public API