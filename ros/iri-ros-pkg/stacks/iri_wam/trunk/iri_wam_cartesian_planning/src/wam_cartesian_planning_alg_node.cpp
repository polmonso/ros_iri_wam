#include "wam_cartesian_planning_alg_node.h"

WamCartesianPlanningAlgNode::WamCartesianPlanningAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<WamCartesianPlanningAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

WamCartesianPlanningAlgNode::~WamCartesianPlanningAlgNode(void)
{
  // [free dynamic memory]
}

void WamCartesianPlanningAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void WamCartesianPlanningAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void WamCartesianPlanningAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<WamCartesianPlanningAlgNode>(argc, argv, "wam_cartesian_planning_alg_node");
}
