#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

typedef  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> ExecutorActionServer;
typedef  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle GoalHandle;	
typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class WAMData
{
  private:
    TrajClient* traj_client_;
    ExecutorActionServer server;
    ros::NodeHandle node_server;
    ros::NodeHandle node_client;
    boost::mutex lock;
    bool has_data;
    
  public:
  
   WAMData(ros::NodeHandle &n):
    node_server(n),
    server(node_server, "iri_wam_controller/follow_joint_trajectory", false),
    has_data(false)
   {
	 ROS_INFO("Enabling Server: Follow_Joint_Trajectory_Action ");
     server.registerGoalCallback(boost::bind(&WAMData::goalCB, this, _1));
     server.start();
     // tell the action client that we want to spin a thread by default
     traj_client_ = new TrajClient("arm_controller/joint_trajectory_action", true);
     // wait for action server to come up
     while(!traj_client_->waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the joint_trajectory_action server");
     }	  
   } 	
 
   void goalCB(GoalHandle goal)
   {
	   
   }	
	
};
