#include "wam_controller_driver_node.h"
using namespace Eigen;

WamControllerDriverNode::WamControllerDriverNode(ros::NodeHandle &nh) : 
  iri_base_driver::IriBaseNodeDriver<WamControllerDriver>(nh),
  follow_joint_trajectory_aserver_(public_node_handle_, "follow_joint_trajectory")
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  //TODO: ask wam for the number of joints
  this->JointState_msg_.name.resize(7);
  this->JointState_msg_.position.resize(7);

  // [init publishers]
  this->libbarrett_link_tcp_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseStamped>("libbarrett_link_tcp", 1);
  this->joint_states_publisher_ = this->public_node_handle_.advertise<sensor_msgs::JointState>("/joint_states", 1);
  
  // [init subscribers]
  
  // [init services]
  this->joints_move_server_ = this->public_node_handle_.advertiseService("joints_move", &WamControllerDriverNode::joints_moveCallback, this);
  
  // [init clients]
  
  // [init action servers]
  follow_joint_trajectory_aserver_.registerStartCallback(boost::bind(&WamControllerDriverNode::follow_joint_trajectoryStartCallback, this, _1)); 
  follow_joint_trajectory_aserver_.registerStopCallback(boost::bind(&WamControllerDriverNode::follow_joint_trajectoryStopCallback, this)); 
  follow_joint_trajectory_aserver_.registerIsFinishedCallback(boost::bind(&WamControllerDriverNode::follow_joint_trajectoryIsFinishedCallback, this)); 
  follow_joint_trajectory_aserver_.registerHasSucceedCallback(boost::bind(&WamControllerDriverNode::follow_joint_trajectoryHasSucceedCallback, this)); 
  follow_joint_trajectory_aserver_.registerGetResultCallback(boost::bind(&WamControllerDriverNode::follow_joint_trajectoryGetResultCallback, this, _1)); 
  follow_joint_trajectory_aserver_.registerGetFeedbackCallback(boost::bind(&WamControllerDriverNode::follow_joint_trajectoryGetFeedbackCallback, this, _1)); 
  follow_joint_trajectory_aserver_.start();
  
  // [init action clients]
  ROS_INFO("Wam node started");

}

void WamControllerDriverNode::mainNodeThread(void)
{
  //lock access to driver if necessary
  //this->driver_.lock();

  std::vector<double> angles(7,0.1);
  std::vector<double> pose(16,0);
  Matrix3f rmat;
// [fill msg Header if necessary]
  std::string robot_name = this->driver_.get_robot_name();
  std::stringstream ss_tcp_link_name;
  ss_tcp_link_name << robot_name << "_link_base";

  this->PoseStamped_msg_.header.stamp = ros::Time::now();
  this->PoseStamped_msg_.header.frame_id = ss_tcp_link_name.str().c_str();

  // [fill msg structures]
  //this->PoseStamped_msg_.data = my_var;
  this->driver_.lock();
  this->driver_.get_pose(&pose);
  this->driver_.get_joint_angles(&angles);
  this->driver_.unlock();

  rmat << pose.at(0), pose.at(1), pose.at(2), pose.at(4), pose.at(5), pose.at(6), pose.at(8), pose.at(9), pose.at(10);

  {
    Quaternion<float> quat(rmat);

    this->PoseStamped_msg_.pose.position.x = pose.at(3);
    this->PoseStamped_msg_.pose.position.y = pose.at(7);
    this->PoseStamped_msg_.pose.position.z = pose.at(11);
    this->PoseStamped_msg_.pose.orientation.x = quat.x();
    this->PoseStamped_msg_.pose.orientation.y = quat.y();
    this->PoseStamped_msg_.pose.orientation.z = quat.z();
    this->PoseStamped_msg_.pose.orientation.w = quat.w();
  }

  JointState_msg_.header.stamp = ros::Time::now();
  for(int i=0;i<(int)angles.size();i++){
      std::stringstream ss_jname;
      ss_jname << robot_name << "_joint_" << i+1;
      JointState_msg_.name[i] = ss_jname.str().c_str();
      JointState_msg_.position[i] = angles[i];
  }
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->libbarrett_link_tcp_publisher_.publish(this->PoseStamped_msg_);
  this->joint_states_publisher_.publish(this->JointState_msg_);

  //unlock access to driver if previously blocked
  this->driver_.unlock();
}

/*  [subscriber callbacks] */

/*  [service callbacks] */
bool WamControllerDriverNode::joints_moveCallback(iri_wam_common_msgs::joints_move::Request &req, iri_wam_common_msgs::joints_move::Response &res) 
{ 
	ROS_INFO("WamControllerDriverNode::joints_moveCallback: New Request Received!"); 

	//use appropiate mutex to shared variables if necessary 
	this->driver_.lock(); 
	//this->joints_move_mutex_.enter(); 

	if(!this->driver_.isRunning()) 
	{ 
		ROS_INFO("WamControllerDriverNode::joints_moveCallback: ERROR: driver is not on run mode yet."); 
		this->driver_.unlock(); 
    res.success = false;
		return false; 
	} 

	//this call blocks if the wam faults. The mutex is not freed...!   
	if ((req.velocities.size() == 0) || (req.accelerations.size() == 0))
	{
		this->driver_.move_in_joints(& req.joints);
	}
	else
	{
		this->driver_.move_in_joints(& req.joints, &req.velocities, &req.accelerations);
	}

	//unlock previously blocked shared variables 
	this->driver_.unlock();
	//this->joints_move_mutex_.exit(); 

	this->driver_.wait_move_end();
  res.success = true;
	return true; 
}

/*  [action callbacks] */


/************       follow_joint_trajectory  ************************/

void WamControllerDriverNode::follow_joint_trajectoryStartCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  ROS_DEBUG("[WamDriverNode]: New FollowJointTrajectoryGoal RECEIVED!"); 
  driver_.lock(); 
    //check goal 
    //execute goal 
  driver_.move_trajectory_in_joints(goal->trajectory);
  driver_.unlock(); 
  ROS_DEBUG("[WamDriverNode]: FollowJointTrajectoryGoal SENT TO ROBOT!");
} 

void WamControllerDriverNode::follow_joint_trajectoryStopCallback(void) 
{ 
  ROS_DEBUG("[WamDriverNode]: New CancelFollowJointTrajectoryAction RECEIVED!");
  driver_.lock();
  driver_.stop_trajectory_in_joints();
  driver_.unlock();
  ROS_DEBUG("[WamDriverNode]: CancelFollowJointTrajectoryAction SENT TO ROBOT!");
} 

bool WamControllerDriverNode::follow_joint_trajectoryIsFinishedCallback(void) 
{ 
  bool ret = false; 
  ROS_DEBUG("[WamDriverNode]: FollowJointTrajectory NOT FINISHED");
  // This sleep "assures" that the robot receives the trajectory and starts moving
  sleep(1);

  driver_.lock();
  if (! driver_.is_moving_trajectory()){
    ret = true;
    ROS_INFO("[WamDriverNode]: FollowJointTrajectory FINISHED!");
  }
  driver_.unlock(); 
  return ret; 
} 

bool WamControllerDriverNode::follow_joint_trajectoryHasSucceedCallback(void) 
{ 
  bool ret = false; 

  driver_.lock(); 
    //if goal was accomplished 
    //ret = true; 
  driver_.unlock(); 

  return ret; 
} 

void WamControllerDriverNode::follow_joint_trajectoryGetResultCallback(control_msgs::FollowJointTrajectoryResultPtr& result) 
{ 
  // MSG has an empty result message
  driver_.lock();
  if (driver_.is_joint_trajectory_result_succeeded()) {
    result->error_code = 0;
  } else {
    result->error_code = -1;
  }
  driver_.unlock(); 
} 

void WamControllerDriverNode::follow_joint_trajectoryGetFeedbackCallback(control_msgs::FollowJointTrajectoryFeedbackPtr& feedback) 
{ 
  driver_.lock(); 
    //keep track of feedback 
  feedback->desired = driver_.get_desired_joint_trajectory_point();
  //ROS_DEBUG("feedback: %s", feedback->data.c_str()); 
  driver_.unlock(); 
}

/**************************************************************/


/*  [action requests] */

void WamControllerDriverNode::postNodeOpenHook(void)
{
}

void WamControllerDriverNode::addNodeDiagnostics(void)
{
}

void WamControllerDriverNode::addNodeOpenedTests(void)
{
}

void WamControllerDriverNode::addNodeStoppedTests(void)
{
}

void WamControllerDriverNode::addNodeRunningTests(void)
{
}

void WamControllerDriverNode::reconfigureNodeHook(int level)
{
}

WamControllerDriverNode::~WamControllerDriverNode(void)
{
  // [free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<WamControllerDriverNode>(argc, argv, "wam_controller_driver_node");
}
