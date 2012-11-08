#include "wam_move_arm_alg.h"
static const std::string FK_SERVICE = "/iri_wam_iri_wam_kinematics/get_fk";
static const std::string INFO_SERVICE = "/iri_wam_iri_wam_kinematics/get_fk_solver_info";
WamMoveArmAlgorithm::WamMoveArmAlgorithm(void)
{


}

WamMoveArmAlgorithm::~WamMoveArmAlgorithm(void)
{
}

void WamMoveArmAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// WamMoveArmAlgorithm Public API
void WamMoveArmAlgorithm::reconfigure_point(arm_navigation_msgs::PositionConstraint &position_constraint)
{
 position_constraint.position.x=position_constraint.position.x-tool_x;
 position_constraint.position.y=position_constraint.position.y-tool_y;
 position_constraint.position.z=position_constraint.position.z-tool_z;
}
void WamMoveArmAlgorithm::reconfigure_points(std::vector<arm_navigation_msgs::PositionConstraint> & position_constraint)
{
	for(size_t i =0; i < position_constraint.size(); ++i)
	{
		reconfigure_point(position_constraint[i]);
	}
	
}
void WamMoveArmAlgorithm::reconfigure_joint(std::vector<arm_navigation_msgs::JointConstraint>& joint_constraints,arm_navigation_msgs::MoveArmGoal& goal)
{
	std::vector<double> position;
	geometry_msgs::PoseStamped pose;
	arm_navigation_msgs::MoveArmGoal move=goal;
	for(size_t i=0; i < joint_constraints.size(); ++i)
	{
		position.push_back(joint_constraints[i].position);
	}
	calculateFK(position,pose);
	makeMsg(pose,move,link_name);
	goal=move;
}
void WamMoveArmAlgorithm::calculateFK(const std::vector<double> &pos,geometry_msgs::PoseStamped& pose)
{
  ros::NodeHandle rh;
  ros::service::waitForService(INFO_SERVICE);
  ros::service::waitForService(FK_SERVICE);
  ros::ServiceClient query_client =   rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo> (INFO_SERVICE);
  ros::ServiceClient fk_client = rh.serviceClient<kinematics_msgs::GetPositionFK>(FK_SERVICE);

  // define the service messages
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;
  if(!query_client.call(request,response))
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(1);
  }
  // define the service messages
  kinematics_msgs::GetPositionFK::Request  fk_request;
  kinematics_msgs::GetPositionFK::Response fk_response;
  fk_request.header.frame_id = "wam_link0";
  fk_request.fk_link_names = response.kinematic_solver_info.link_names;
  fk_request.robot_state.joint_state.position=pos;
  fk_request.robot_state.joint_state.name =  response.kinematic_solver_info.joint_names;
  link_name=response.kinematic_solver_info.link_names[response.kinematic_solver_info.link_names.size()-1];
  if(fk_client.call(fk_request, fk_response))
  {
    if(fk_response.error_code.val == fk_response.error_code.SUCCESS)
    {
		pose=fk_response.pose_stamped[fk_response.pose_stamped.size()-1];
    }
    else
    {
      ROS_ERROR("Forward kinematics failed");
     ROS_ERROR_STREAM("CODE: "<<fk_response.error_code.val);
    }
  }
  else
  {
    ROS_ERROR("Forward kinematics service call failed");
       ros::shutdown();
    exit(1);
  }
}
void WamMoveArmAlgorithm::makeMsg(const geometry_msgs::PoseStamped &pose, arm_navigation_msgs::MoveArmGoal& goal, const std::string& link)
{
	arm_navigation_msgs::PositionConstraint position_constraint;
    arm_navigation_msgs::OrientationConstraint orientation_constraints;
	arm_navigation_msgs::poseStampedToPositionOrientationConstraints(pose,link,position_constraint,orientation_constraints);
	reconfigure_point(position_constraint);
    goal.motion_plan_request.goal_constraints.position_constraints.clear();
    goal.motion_plan_request.goal_constraints.orientation_constraints.clear();
    goal.motion_plan_request.goal_constraints.joint_constraints.clear();
    goal.motion_plan_request.goal_constraints.position_constraints.push_back(position_constraint);  
    goal.motion_plan_request.goal_constraints.orientation_constraints.push_back(orientation_constraints);
}
bool WamMoveArmAlgorithm::hasPose(const arm_navigation_msgs::MoveArmGoal& goal)
{
	return (goal.motion_plan_request.goal_constraints.position_constraints.size() == 0)? false: true;
}
bool WamMoveArmAlgorithm::hasJoint(const arm_navigation_msgs::MoveArmGoal& goal)
{
	return (goal.motion_plan_request.goal_constraints.joint_constraints.size() == 0)? false: true;
}
void WamMoveArmAlgorithm::reconfigure(arm_navigation_msgs::MoveArmGoal& goal)
{
	if(hasPose(goal))
    reconfigure_points(goal.motion_plan_request.goal_constraints.position_constraints);
	else if(hasJoint(goal))
    reconfigure_joint(goal.motion_plan_request.goal_constraints.joint_constraints,goal);
}
