#include "wam_move_arm_alg.h"

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
void WamMoveArmAlgorithm::reconfigure_joint(std::vector<arm_navigation_msgs::JointConstraint>& joint_constraints)
{
	
}
void WamMoveArmAlgorithm::calculateFK(const std::vector<double> &pos,const std::string& frame)
{
  ros::NodeHandle rh;
  ros::service::waitForService("pr2_right_arm_kinematics/get_fk_solver_info");
  ros::service::waitForService("pr2_right_arm_kinematics/get_fk");
  ros::ServiceClient query_client =   rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo> ("pr2_right_arm_kinematics/get_fk_solver_info");
  ros::ServiceClient fk_client = rh.serviceClient<kinematics_msgs::GetPositionFK>("pr2_right_arm_kinematics/get_fk");

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
  fk_request.header.frame_id = frame;
  link_name = response.kinematic_solver_info.link_names[0];
  fk_request.fk_link_names = link_name;
  fk_request.robot_state.joint_state.position=pos;
  fk_request.robot_state.joint_state.name =  response.kinematic_solver_info.joint_names;
  
  if(fk_client.call(fk_request, fk_response))
  {
    if(fk_response.error_code.val == fk_response.error_code.SUCCESS)
    {
		pose=fk_response.pose_stamped[fk_response.pose_stamped.size()-1];
    }
    else
    {
      ROS_ERROR("Forward kinematics failed");
         ros::shutdown();
    exit(1);
    }
  }
  else
  {
    ROS_ERROR("Forward kinematics service call failed");
       ros::shutdown();
    exit(1);
  }
}
void WamMoveArmAlgorithm::makeMsg(const geometry_msgs::PoseStamped &pose, arm_navigation_msgs::MoveArmGoal& goal, const std::string& link);
{
	arm_navigation_msgs::PositionConstraint position_constraint;
    arm_navigation_msgs::OrientationConstraint orientation_constraint;
	poseStampedToPositionOrientationConstraints(pose,link,position_constraint,orientation_constraint,0.01,0.01);
	reconfigure_point(position_constraint);
    goal.motion_plan_request.goal_constraints.position_constraints.push_back(position_constraint);
    goal.motion_plan_request.goal_constraints.orientation_constraints.push_back(orientation_constraints);
}
