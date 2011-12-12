#ifndef WAM_GAZEBO_HH
#define WAM_GAZEBO_HH

#include <ros/ros.h>
#include "iri_wam_common_msgs/joints_move.h"
class WAMGazebo
{
	
	public:
	
	 WAMGazebo();
	 ~WAMGazebo();
	 joints_moveCB(iri_wam_common_msgs::joints_move::Request &req, iri_wam_common_msgs::joints_move::Response &res);
	 
	private:
	actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > client_trajectory_;
	ros::NodeHandle node;
    ros::ServiceServer service_;
    std::vector<std::string> names_joints;    
	//this->joints_move_server = this->public_node_handle_.advertiseService("joints_move", &WamDriverNode::joints_moveCallback, this);
};


#endif
