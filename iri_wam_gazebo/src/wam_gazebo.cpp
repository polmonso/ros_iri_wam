#include "wam_gazebo.h"

static const std:string joints_move="joints_move";


WAMGazebo::WAMGazebo()
 {
	service_= node.advertiseService("joints_move", &WAMGazebo::joints_moveCB, this);
	names_joints.push_back("j1");
	names_joints.push_back("j2");
	names_joints.push_back("j3");
	names_joints.push_back("j4");
	names_joints.push_back("j5");
	names_joints.push_back("j6");
	names_joints.push_back("j7");
	client_trajectory_ = new TrajClient("iri_wam_controller/joint_trajectory_action", true);
    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }

 }
 WAMGazebo::~WAMGazebo()
 {
	 delete client_trajectory_;
 }
 
 bool WAMGazebo::joints_moveCB(iri_wam_common_msgs::joints_move::Request &req, iri_wam_common_msgs::joints_move::Response &res)
 {
	 
 }
