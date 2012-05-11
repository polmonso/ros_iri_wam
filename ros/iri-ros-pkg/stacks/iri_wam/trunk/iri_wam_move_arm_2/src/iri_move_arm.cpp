#include "iri_wam_move_arm/iri_move_arm.h"
IriMoveArm::IriMoveArm():
root_handle("/"),
private_handle("~")
{	
	// tell the action client that we want to spin a thread by default
	traj_client_ = new TrajClient("/syn_move_iri_wam", true);
	// wait for action server to come up
	while(!traj_client_->waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the syn_move_iri_wam server");
	}
	
	time_pub_  = root_handle.advertise<std_msgs::Duration>("/move_iri_wam/time_path", 10);
    action_server_.reset(new actionlib::SimpleActionServer<arm_navigation_msgs::MoveArmAction>(root_handle, "move_iri_wam", boost::bind(&IriMoveArm::move_arm, this, _1), false));
    action_server_->start();
}

IriMoveArm::~IriMoveArm()
 {
	 delete traj_client_;
 }

void IriMoveArm::move_arm(const arm_navigation_msgs::MoveArmGoalConstPtr& goal)
{
	ros::Duration path_time;
	getTime(*goal,path_time);
	sendTime(path_time);
	sendTraj(*goal);
}

void IriMoveArm::getTime(const arm_navigation_msgs::MoveArmGoal& msg, ros::Duration& time)
{
 time = msg.motion_plan_request.expected_path_dt;
}

 void IriMoveArm::sendTime(const ros::Duration& time)
 {
	 std_msgs::Duration dt;
	 dt.data=time;
	 time_pub_.publish(dt);	 
 }
 
 void IriMoveArm::sendTraj(const arm_navigation_msgs::MoveArmGoal& goal)
 {
	 traj_client_->sendGoal(goal);
 }
 
 int main(int argc,char ** argv)
 {
	ros::init(argc,argv,"iri_wam_move_arm");
	 IriMoveArm wam;
	 ros::spin();	 
 }
