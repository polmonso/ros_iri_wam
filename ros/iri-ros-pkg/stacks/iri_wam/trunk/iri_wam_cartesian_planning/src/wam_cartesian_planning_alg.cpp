#include "wam_cartesian_planning_alg.h"

WamCartesianPlanningAlgorithm::WamCartesianPlanningAlgorithm(void)
{
}

WamCartesianPlanningAlgorithm::~WamCartesianPlanningAlgorithm(void)
{
}

void WamCartesianPlanningAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

bool WamCartesianPlanningAlgorithm::loadModel(const std::string& model)
{
	if(!collada_urdf::colladaFromUrdfString(model, dom)) 
	{
      ROS_ERROR("Failed to construct COLLADA DOM");
      return false;
	}
	return true;
}
 bool WamCartesianPlanningAlgorithm::toFile(const std::string& pathDae)
 {
	 if(dom) std::string str="@";
 }
// WamCartesianPlanningAlgorithm Public API


/*
colladaFromUrdfString

collada_urdf::colladaToFile(dom, "filename.dae");



ros::NodeHandle node;

node.param("robot_description", robot_desc_string, std::string());






*/
