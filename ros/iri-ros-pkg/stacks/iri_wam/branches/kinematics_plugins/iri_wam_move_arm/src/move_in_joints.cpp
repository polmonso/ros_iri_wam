#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <sstream>
#include <iostream>
#include <stdlib.h>

typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> TrajClient;

class MoveInJoints
{
  private:
	// Action client for the joint trajectory action 
    // used to trigger the   arm movement action
	TrajClient* traj_client_;
	ros::Duration time_move;
	std::vector<double> pos;
	std::vector<std::string> names;
  public:
   //! Initialize the action client and wait for action server to come up
	MoveInJoints() 
	{
		// tell the action client that we want to spin a thread by default
		traj_client_ = new TrajClient("/move_iri_wam", true);
		
		// wait for action server to come up
		while(!traj_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_iri_wam server");
		}
	}
	//! Clean up the action client
	~MoveInJoints()
	{
     delete traj_client_;
	}
	//! Push joint name on vector 
	void setNamesJoint(void)
	{
		names.push_back("j1_joint");
		names.push_back("j2_joint");
		names.push_back("j3_joint");
		names.push_back("j4_joint");
		names.push_back("j5_joint");
		names.push_back("j6_joint");
		names.push_back("j7_joint");
	}
	//! Sends the command to start a given trajectory
    void startTrajectory(arm_navigation_msgs::MoveArmGoal& goal)
    {
        traj_client_->sendGoal(goal);
    }
    //! Get move from keyboard
    void getTrajectory(int argc, char** argv)
	{
	 if(argc < 8 || argc > 9) 
	 {
	   ROS_FATAL("Error: The numbers of parameters out of bound "); 
	   exit(1);
	 }
	 pos.resize(7);
	 for(int i =1; i <=7; ++i) pos[i-1]=strtod(argv[i],NULL);

	 time_move= (argc == 9)?ros::Duration(strtod(argv[8],NULL)):ros::Duration(0.0);
	}  
    //! Returns the current state of the action
    actionlib::SimpleClientGoalState getState()
    {
     return traj_client_->getState();
    }	  
    //!Defined Motion Request Parameters
	void setPlannerRequest(arm_navigation_msgs::MoveArmGoal& goal)
	{
		goal.motion_plan_request.group_name="iri_wam";
		goal.motion_plan_request.num_planning_attempts = 1;
		goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
		goal.motion_plan_request.planner_id = std::string("");
		goal.planner_service_name=std::string("ompl_planning/plan_kinematic_path");
		goal.motion_plan_request.expected_path_dt = time_move;
	}
	//!Defined Goal Constraint Parameters
	void setPlannerRequestGoalConstraint(arm_navigation_msgs::MoveArmGoal& goal)
	{ 
	  goal.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());
	  for (unsigned int i = 0 ; i < names.size(); ++i)
	  {
       goal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
       goal.motion_plan_request.goal_constraints.joint_constraints[i].position = pos[i];
       goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = pos[i]+ 0.0119999;
	   goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = pos[i] -0.0119999;
	   if(goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below < 0.0){
		goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below *= -1;
	   }
	   if(goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above < 0.0){
		goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above *= -1;
	   }
	  }
	}
};
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "move_arm_node");
  MoveInJoints arm;
  arm_navigation_msgs::MoveArmGoal move;
  // Start the trajectory  
  arm.getTrajectory(argc,argv);
  arm.setNamesJoint();
  arm.setPlannerRequest(move);
  arm.setPlannerRequestGoalConstraint(move);
  arm.startTrajectory(move);
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
	  ROS_DEBUG("WAIT To END TRAJECTORY");
  }
      
 bool success = (arm.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  if(success)
   ROS_INFO("Action finished: %s",arm.getState().toString().c_str());
  else
   ROS_INFO("Action failed: %s",arm.getState().toString().c_str());
}
