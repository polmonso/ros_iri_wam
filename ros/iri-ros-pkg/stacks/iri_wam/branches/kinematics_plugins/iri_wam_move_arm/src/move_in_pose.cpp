#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <sstream>
#include <iostream>
#include <stdlib.h>

typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> TrajClient;
class MoveInPose
{
  private:
	// Action client for the joint trajectory action 
    // used to trigger the arm movement action
	TrajClient* traj_client_;
	ros::Duration time_move;
	std::vector<double> posicion;
	std::vector<double> rotacion;
	std::vector<std::string> names;
  public:
   //! Initialize the action client and wait for action server to come up
	MoveInPose() 
	{
		// tell the action client that we want to spin a thread by default
		traj_client_ = new TrajClient("move_iri_wam", true);
		
		// wait for action server to come up
		while(!traj_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_iri_wam server");
		}
	}
	//! Clean up the action client
	~MoveInPose()
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
	 posicion.resize(3);
	 rotacion.resize(4);
	 for(int i =1; i <=7; ++i) 
	 {
	  (i < 4)? posicion[i-1]=strtod(argv[i],NULL):rotacion[i-4]=strtod(argv[i],NULL);
     }
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
	void setPlannerRequestPose(arm_navigation_msgs::MoveArmGoal& goal)
	{ 
		arm_navigation_msgs::SimplePoseConstraint desired_pose;
		/*ros::NodeHandle nh("/");
	    
		if(!nh.hasParam("/move_arm/tip_name"))
		{
			ROS_FATAL("tip_name undefined");
			exit(1);
		} 	
		std::string tip_name="";	 
		nh.param("/move_arm/tip_name", desired_pose.link_name, tip_name);
		if(!nh.hasParam("/move_arm/root_name"))
		{
			ROS_FATAL("root_name undefined");
			exit(1);
		} 
		std::string root_name="";
		nh.param("/move_arm/root_name", desired_pose.header.frame_id,root_name);

		*/
		desired_pose.header.frame_id="world";
		desired_pose.link_name="lk_wam7";
		desired_pose.pose.position.x = posicion[0];
		desired_pose.pose.position.y = posicion[1];
		desired_pose.pose.position.z = posicion[2];

		
		desired_pose.pose.orientation.x = rotacion[0];
		desired_pose.pose.orientation.y = rotacion[1];
		desired_pose.pose.orientation.z = rotacion[2];
		desired_pose.pose.orientation.w = rotacion[3];
		
		/*double tolerance_x;
		double tolerance_y;
		double tolerance_z;
		double tolerance_roll;
		double tolerance_pitch;
		double tolerance_yaw;
		
		nh.param("/move_arm/position/tolerance_x", tolerance_x, 0.02);
		nh.param("/move_arm/position/tolerance_y", tolerance_y, 0.02);
		nh.param("/move_arm/position/tolerance_z", tolerance_z, 0.02);
		nh.param("/move_arm/rotation/tolerance_roll", tolerance_roll, 0.04);
		nh.param("/move_arm/rotation/tolerance_pitch", tolerance_pitch,0.04);
		nh.param("/move_arm/rotation/tolerance_yaw", tolerance_yaw, 0.04);*/
				
		desired_pose.absolute_position_tolerance.x = 0.02;
		desired_pose.absolute_position_tolerance.y = 0.02;
		desired_pose.absolute_position_tolerance.z = 0.02;
		
		desired_pose.absolute_roll_tolerance = 0.04;
		desired_pose.absolute_pitch_tolerance = 0.04;
		desired_pose.absolute_yaw_tolerance = 0.04;
		
		arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goal);
	}
};
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "move_arm_node");
  MoveInPose arm;
  arm_navigation_msgs::MoveArmGoal move;
  // Start the trajectory  
  arm.getTrajectory(argc,argv);
  arm.setNamesJoint();
  arm.setPlannerRequest(move);
  arm.setPlannerRequestPose(move);
  arm.startTrajectory(move);
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
      
 bool success = (arm.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  if(success)
   ROS_INFO("Action finished: %s",arm.getState().toString().c_str());
  else
   ROS_INFO("Action failed: %s",arm.getState().toString().c_str());
}
