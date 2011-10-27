#include "tibi_dabo_hri_alg.h"

TibiDaboHriAlgorithm::TibiDaboHriAlgorithm()
{
}

TibiDaboHriAlgorithm::~TibiDaboHriAlgorithm()
{
}

void TibiDaboHriAlgorithm::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// TibiDaboHriAlgorithm Public API
