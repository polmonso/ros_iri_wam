#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Duration.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
typedef actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction> JointExecutorActionClient;
typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JointExecutorActionServer;
typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> FJTAS;
typedef FJTAS::GoalHandle GoalHandleFollow;

class IriMoveController
{
	
	public:
	
  IriMoveController();
  ~IriMoveController();
  
  
  private:
  
  ros::NodeHandle root_handle;
  ros::NodeHandle private_handle;
  
  JointExecutorActionClient executorClient;
  JointExecutorActionServer executorServer;

  void sendTraj();
  void receiveTraj(GoalHandleFollow gh);


};
