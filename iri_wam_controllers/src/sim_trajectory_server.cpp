#include "iri_wam_controllers/sim_trajectory_server.h"

namespace sim_trajectory_server
{
 sim_trajectory_server::sim_trajectory_server():
 private_hand_("~")
 { 
	bringupTopics(); 
	bringupActions();
	private_hand_.param<double>("waited_time_for_result", waited_time_for_result,200);

 }

 
void sim_trajectory_server::bringupActions()
{
   // actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> c_C(private_hand_,"bb",true);
    action_client_= new actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction>(public_hand_,"syn_move_arm",true);
    action_client_->waitForServer();
    ROS_INFO("Initializer Client Action : syn_move_arm");
	action_server_.reset(new actionlib::SimpleActionServer<arm_navigation_msgs::MoveArmAction>(public_hand_,"move_iri_wam", boost::bind(&sim_trajectory_server::goalCB, this, _1), false));
    action_server_->start();	
    ROS_INFO("Initializer Server Action : move_iri_wam");
}
void sim_trajectory_server::bringupTopics()
{
	duration_path_pub_=private_hand_.advertise<iri_wam_controllers::PathDuration>("duration_path", 5);	
}
void sim_trajectory_server::goalCB(const arm_navigation_msgs::MoveArmGoalConstPtr& goal)
{
	TimetoMsg(goal);
	sendTrajectory(goal);
}

void sim_trajectory_server::TimetoMsg(const arm_navigation_msgs::MoveArmGoalConstPtr& goal)
{
   iri_wam_controllers::PathDuration T;
   T.duration_path_expected=goal->motion_plan_request.expected_path_duration;
   sendTime(T);
}

void sim_trajectory_server::sendTime(const iri_wam_controllers::PathDuration& msg)
{
	duration_path_pub_.publish(msg);
}
void sim_trajectory_server::sendTrajectory(const arm_navigation_msgs::MoveArmGoalConstPtr& goal)
{
	arm_navigation_msgs::MoveArmGoal goalA;
	goalA=*goal;
	if (private_hand_.ok())
	{
		bool finished_within_time = false;
		action_client_->sendGoal(goalA);
		finished_within_time = action_client_->waitForResult(ros::Duration(waited_time_for_result));
		if (!finished_within_time)
		{
			action_client_->cancelGoal();
			ROS_INFO("Timed out achieving goal A");
		}
		else
		{
			actionlib::SimpleClientGoalState state = action_client_->getState();
			bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
			if(success)
				ROS_INFO("Action finished: %s",state.toString().c_str());
			else
				ROS_INFO("Action failed: %s",state.toString().c_str());
		}
	} 
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wam_move_arm");
  
 // ros::AsyncSpinner spinner(1); // Use 1 thread
  //spinner.start();
  //std::string group;
sim_trajectory_server::sim_trajectory_server server;
  
  ROS_INFO("Move arm action started");
 // ros::waitForShutdown();
   ros::spin(); 
   
  return 0;
}
 
 /***

  ros::SubscribeOptions so =ros::SubscribeOptions::create<trajectory_msgs::JointTrajectory>("command", 1,
  																boost::bind(&WAMGazebo::topicCommand, this, _1),ros::VoidPtr(), &queue_);  																
  sub_command= rosnode_->subscribe(so);
  state_publisher=rosnode_->advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("state", 5);	
***/
