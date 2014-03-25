#include "wam_arm_navigation_alg.h"

WamArmNavigationAlgorithm::WamArmNavigationAlgorithm(void):
frame_id_target(""),
has_pose(false)
{
}

WamArmNavigationAlgorithm::~WamArmNavigationAlgorithm(void)
{
}

void WamArmNavigationAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// WamArmNavigationAlgorithm Public API
void WamArmNavigationAlgorithm::setTarget(const geometry_msgs::PoseStamped& pose,const std::string& nameFrame)
{
	frame_id_target=nameFrame;
	pose_goal=pose;
	has_pose=true;
}
bool WamArmNavigationAlgorithm::hasPose()
{
	return has_pose;
}
void WamArmNavigationAlgorithm::sendedPose()
{
	has_pose=false;
}
//!Defined Motion Request Parameters
void WamArmNavigationAlgorithm::setPlannerRequest(arm_navigation_msgs::MoveArmGoal& goal)
{
	goal.planner_service_name=std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.group_name="iri_wam";
	goal.motion_plan_request.num_planning_attempts = 2;
	goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
	goal.motion_plan_request.planner_id = std::string("");
	goal.motion_plan_request.expected_path_duration = ros::Duration(10.0);
	goal.motion_plan_request.expected_path_dt = ros::Duration(0.2);
}

//!Defined Goal Constraint Parameters
void WamArmNavigationAlgorithm::setPlannerRequestPose(arm_navigation_msgs::MoveArmGoal& goal)
{ 
	
	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.link_name = frame_id_target;
	desired_pose.header.frame_id = pose_goal.header.frame_id;
	desired_pose.pose = pose_goal.pose;
  desired_pose.absolute_position_tolerance.x = 0.05;
  desired_pose.absolute_position_tolerance.y = 0.05;
  desired_pose.absolute_position_tolerance.z = 0.05;
  desired_pose.absolute_roll_tolerance = 0.05;
  desired_pose.absolute_pitch_tolerance = 0.05;
  desired_pose.absolute_yaw_tolerance = 0.05;
	
	addGoalConstraintToMoveArmGoal(desired_pose, goal);
	
}

/**
 * 
 * 
 **/ 
void WamArmNavigationAlgorithm::addGoalConstraintToMoveArmGoal(const arm_navigation_msgs::SimplePoseConstraint &pose_constraint, arm_navigation_msgs::MoveArmGoal &move_arm_goal)
{
  arm_navigation_msgs::PositionConstraint position_constraint;
  arm_navigation_msgs::OrientationConstraint orientation_constraint;
  arm_navigation_msgs::poseConstraintToPositionOrientationConstraints(pose_constraint, position_constraint, orientation_constraint);
  move_arm_goal.motion_plan_request.goal_constraints.position_constraints.push_back(position_constraint);
  move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.push_back(orientation_constraint);
}
