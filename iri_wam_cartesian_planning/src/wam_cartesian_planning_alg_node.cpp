#include "wam_cartesian_planning_alg_node.h"

WamCartesianPlanningAlgNode::WamCartesianPlanningAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<WamCartesianPlanningAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  this->path_planning_server_ = this->public_node_handle_.advertiseService("path_planning", &WamCartesianPlanningAlgNode::path_planningCallback, this);
  
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
bool WamCartesianPlanningAlgNode::path_planningCallback(iri_wam_cartesian_planning::PosePath::Request &req, iri_wam_cartesian_planning::PosePath::Response &res) 
{ 
  ROS_INFO("WamCartesianPlanningAlgNode::path_planningCallback: New Request Received!"); 
  std::vector<geometry_msgs::PoseStamped> vector;
  //use appropiate mutex to shared variables if necessary 
  this->alg_.lock(); 
	ompl::base::ScopedState<ompl::base::SE3StateSpace> st1 =rosToOmpl(req.init_pose);
	ompl::base::ScopedState<ompl::base::SE3StateSpace> st2 =rosToOmpl(req.goal_pose);
	this->alg_.planWithSimpleSetup(st1,st2,req.states,vector);
	res.poses_solution=vector;
	res.solution_found=(vector.size() >0)? true: false;
	this->alg_.unlock(); 
  return true; 
}

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

ompl::base::ScopedState<ompl::base::SE3StateSpace> WamCartesianPlanningAlgNode::rosToOmpl(const geometry_msgs::PoseStamped& msg)
{
  ompl::base::ScopedState<ompl::base::SE3StateSpace> st(this->alg_.space);	
  st->setXYZ(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
  st->rotation().x=msg.pose.orientation.x;
  st->rotation().y=msg.pose.orientation.y;
  st->rotation().z=msg.pose.orientation.z;
  st->rotation().w=msg.pose.orientation.w;
  return st;
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<WamCartesianPlanningAlgNode>(argc, argv, "wam_cartesian_planning_alg_node");
}
