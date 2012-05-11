#include <ros/ros.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <std_msgs/Duration.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> TrajClient;
class IriMoveArm
{
	
  public:
  IriMoveArm();
  ~IriMoveArm();
  
  private:
  
  ros::NodeHandle root_handle;
  ros::NodeHandle private_handle;
  ros::Publisher  time_pub_;
  
  
  TrajClient* traj_client_;
  
  boost::shared_ptr<actionlib::SimpleActionServer<arm_navigation_msgs::MoveArmAction> > action_server_;	
  
  void move_arm(const arm_navigation_msgs::MoveArmGoalConstPtr& goal);
  void getTime(const arm_navigation_msgs::MoveArmGoal& msg, ros::Duration& time);
  void sendTime(const ros::Duration& time);
  void sendTraj(const arm_navigation_msgs::MoveArmGoal& goal);

};
