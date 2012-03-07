#ifndef _SIM_TRAJECTORY_FILTER_H_
#define _SIM_TRAJECTORY_FILTER_H_

#include <ros/ros.h>

#include <actionlib/server/action_server.h>
#include <actionlib/client/action_client.h>
#include "iri_wam_controllers/PathDuration.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
namespace sim_trajectory_filter
{
 class sim_trajectory_filter 
 {
  // Action typedefs for the new follow joint trajectory action
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> FJTAS;
  typedef FJTAS::GoalHandle GoalHandleFollow;	
  typedef actionlib::ActionClient<pr2_controllers_msgs::JointTrajectoryAction> JointExecutorActionClient;
	private:
		ros::NodeHandle public_hand_;
		ros::NodeHandle private_hand_;
		ros::Subscriber duration_path_sub_;	
		boost::scoped_ptr<FJTAS> action_server_follow_;
		bool recieve_time;
		std::string id_path;
		std::string controller_name;
		ros::Duration time_for_path;
		JointExecutorActionClient* controller_action_client_;
		
		void durationCB(const iri_wam_controllers::PathDuration::ConstPtr& msg);
		void goalCBFollow(GoalHandleFollow gh);
		void cancelCBFollow(GoalHandleFollow gh);
		/**
		 * Dtime use time_for_path variable
		 **/ 
        void get_Dtime(const int& num_points,double& dtime);
        void timeToMsg(trajectory_msgs::JointTrajectory& msg, const double& time);
		void calculateVelocity(const double& p2,const double& p1,const double& dt, double& vel);
		void calculateAccel(const double& v2,const double& v1,const double& dt, double& accel);
		void velocityToMsg(trajectory_msgs::JointTrajectory& msg,const double& dt);		
		void accelToMsg(trajectory_msgs::JointTrajectory& msg,const double& dt);
		void getPositionMsg(const trajectory_msgs::JointTrajectory& msg,const int& index,const int& indexJoint,double& pos);
		void getVelocityMsg(const trajectory_msgs::JointTrajectory& msg,const int& index,const int& indexJoint,double& vel);
		void sendGoal(const pr2_controllers_msgs::JointTrajectoryGoal& msg);
		void copyMsg(const trajectory_msgs::JointTrajectory& a,trajectory_msgs::JointTrajectory& b);
		void sendTrajectory(const trajectory_msgs::JointTrajectory& msg);
	public:
		sim_trajectory_filter();
		~sim_trajectory_filter(){};
		void bringupActions();
		void bringupSubcribers();	
	
	
	
	
	
	
	
 };
}
#endif
