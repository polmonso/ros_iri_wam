#ifndef _SIM_TRAJECTORY_SERVER_H_
#define _SIM_TRAJECTORY_SERVER_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include "iri_wam_controllers/PathDuration.h"
namespace sim_trajectory_server
{
class sim_trajectory_server 
{
	private:
		ros::NodeHandle public_hand_;
		ros::NodeHandle private_hand_;
		ros::Publisher duration_path_pub_;		
		double waited_time_for_result;
		boost::shared_ptr<actionlib::SimpleActionServer <arm_navigation_msgs::MoveArmAction> > action_server_;
		actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> *action_client_;
		
	public:
		sim_trajectory_server();
		~sim_trajectory_server(){};
		void bringupActions();
		void bringupTopics();
		void goalCB(const arm_navigation_msgs::MoveArmGoalConstPtr& goal);
		void cancelCB();
		void sendTrajectory(const arm_navigation_msgs::MoveArmGoalConstPtr& goal);
		void sendTime(const iri_wam_controllers::PathDuration& msg);
		void TimetoMsg(const arm_navigation_msgs::MoveArmGoalConstPtr& goal);

};
}
#endif
