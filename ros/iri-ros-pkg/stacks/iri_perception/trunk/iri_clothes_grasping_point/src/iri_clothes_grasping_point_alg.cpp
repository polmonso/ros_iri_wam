#include "iri_clothes_grasping_point_alg.h"

IriClothesGraspingPointAlgorithm::IriClothesGraspingPointAlgorithm()
{
}

IriClothesGraspingPointAlgorithm::~IriClothesGraspingPointAlgorithm()
{
}

void IriClothesGraspingPointAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  ROS_INFO("Current config: %f",new_cfg.a);
  
  this->unlock();
}

// IriClothesGraspingPointAlgorithm Public API
