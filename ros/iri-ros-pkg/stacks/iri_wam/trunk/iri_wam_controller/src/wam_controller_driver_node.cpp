#include "wam_controller_driver_node.h"
#include "wam_exceptions.h"

using namespace Eigen;

WamControllerDriverNode::WamControllerDriverNode(ros::NodeHandle &nh) : 
  iri_base_driver::IriBaseNodeDriver<WamControllerDriver>(nh),
  dmp_joint_tracker_aserver_(public_node_handle_, "dmp_joint_tracker"),
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
  this->DMPTrackerNewGoal_subscriber_ = this->public_node_handle_.subscribe("DMPTrackerNewGoal", 1, &WamControllerDriverNode::DMPTrackerNewGoal_callback, this);
  
  // [init services]
  this->hold_on_server_ = this->public_node_handle_.advertiseService("hold_on", &WamControllerDriverNode::hold_onCallback, this);
  this->joints_move_server_ = this->public_node_handle_.advertiseService("joints_move", &WamControllerDriverNode::joints_moveCallback, this);
  
  // [init clients]
  
  // [init action servers]
  dmp_joint_tracker_aserver_.registerStartCallback(boost::bind(&WamControllerDriverNode::dmp_joint_trackerStartCallback, this, _1)); 
  dmp_joint_tracker_aserver_.registerStopCallback(boost::bind(&WamControllerDriverNode::dmp_joint_trackerStopCallback, this)); 
  dmp_joint_tracker_aserver_.registerIsFinishedCallback(boost::bind(&WamControllerDriverNode::dmp_joint_trackerIsFinishedCallback, this)); 
  dmp_joint_tracker_aserver_.registerHasSucceedCallback(boost::bind(&WamControllerDriverNode::dmp_joint_trackerHasSucceedCallback, this)); 
  dmp_joint_tracker_aserver_.registerGetResultCallback(boost::bind(&WamControllerDriverNode::dmp_joint_trackerGetResultCallback, this, _1)); 
  dmp_joint_tracker_aserver_.registerGetFeedbackCallback(boost::bind(&WamControllerDriverNode::dmp_joint_trackerGetFeedbackCallback, this, _1)); 
  dmp_joint_tracker_aserver_.start();

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
  //this->driver_.unlock();
}

/*  [subscriber callbacks] */
void WamControllerDriverNode::DMPTrackerNewGoal_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg) 
{ 
  ROS_INFO("WamControllerDriverNode::DMPTrackerNewGoal_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  this->driver_.lock(); 
  //this->DMPTrackerNewGoal_mutex_.enter(); 

  //driver_.dmp_tracker_new_goal(&msg->positions); 

  //std::cout << msg->data << std::endl; 

  //unlock previously blocked shared variables 
  this->driver_.unlock(); 
  //this->DMPTrackerNewGoal_mutex_.exit(); 
}

/*  [service callbacks] */
bool WamControllerDriverNode::hold_onCallback(iri_wam_common_msgs::wamholdon::Request &req, iri_wam_common_msgs::wamholdon::Response &res) 
{ 
    ROS_INFO("WamControllerDriverNode::hold_onCallback: New Request Received!"); 

    //use appropiate mutex to shared variables if necessary 
    this->driver_.lock(); 
    //this->hold_on_mutex_.enter(); 
    if(!this->driver_.isRunning()) 
    { 
        ROS_INFO("WamControllerDriverNode::hold_onCallback: ERROR: driver is not on run mode yet."); 
        this->driver_.unlock(); 
        res.success = false;
        return false;
    } 
    //unlock previously blocked shared variables 
    switch (req.holdon)
    {
        case HOLDON:
            this->driver_.hold_on();
            break;
        case HOLDOFF:
            this->driver_.hold_off();
            break;
        default:
            ROS_ERROR("WamControllerDriverNode::hold_onCallback: ERROR: Invalid service call."); 
            break; 
    }
    this->driver_.unlock(); 
    //this->hold_on_mutex_.exit(); 
    res.success = true;
    return true; 
}

bool WamControllerDriverNode::joints_moveCallback(iri_wam_common_msgs::joints_move::Request &req, iri_wam_common_msgs::joints_move::Response &res) 
{ 
    ROS_INFO("WamControllerDriverNode::joints_moveCallback: New Request Received!"); 
    ROS_INFO("Vel: %f, Acc: %f", req.velocity, req.acceleration); 

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

    if (!this->driver_.move_in_joints(&req.joints, req.velocity, req.acceleration))
    {
        ROS_INFO("WamControllerDriverNode::joints_moveCallback: ERROR: movement not possible.");
        this->driver_.unlock();
        res.success = false;
        return false;
    }

    //unlock previously blocked shared variables 
    this->driver_.unlock();
    //this->joints_move_mutex_.exit(); 
    this->driver_.wait_move_end();
    res.success = true;
    return true;
}

/*  [action callbacks] */

/************************  DMP_joint_tracker  ************************/

void WamControllerDriverNode::dmp_joint_trackerStartCallback(const iri_wam_common_msgs::DMPTrackerGoalConstPtr& goal)
{ 
  driver_.lock(); 
    //check goal 
    //execute goal 
  driver_.start_dmp_tracker(&goal->initial.positions, &goal->goal.positions);
  driver_.unlock(); 
} 

void WamControllerDriverNode::dmp_joint_trackerStopCallback(void) 
{ 
  driver_.lock(); 
    //stop action 
  driver_.unlock(); 
} 

bool WamControllerDriverNode::dmp_joint_trackerIsFinishedCallback(void) 
{ 
  bool ret = false; 

  driver_.lock(); 
    //if action has finish for any reason 
    //ret = true; 
  driver_.unlock(); 

  return ret; 
} 

bool WamControllerDriverNode::dmp_joint_trackerHasSucceedCallback(void) 
{ 
  bool ret = false; 

  driver_.lock(); 
    //if goal was accomplished 
    //ret = true; 
  driver_.unlock(); 

  return ret; 
} 

void WamControllerDriverNode::dmp_joint_trackerGetResultCallback(iri_wam_common_msgs::DMPTrackerResultPtr& result) 
{ 
  driver_.lock(); 
    //update result data to be sent to client 
    //result->data = data; 
  driver_.unlock(); 
} 

void WamControllerDriverNode::dmp_joint_trackerGetFeedbackCallback(iri_wam_common_msgs::DMPTrackerFeedbackPtr& feedback) 
{ 
  driver_.lock(); 
    //keep track of feedback 
    //ROS_INFO("feedback: %s", feedback->data.c_str()); 
  driver_.unlock(); 
}


/************************  follow_joint_trajectory  ************************/

void WamControllerDriverNode::follow_joint_trajectoryStartCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
    ROS_DEBUG("[WamDriverNode]: New FollowJointTrajectoryGoal RECEIVED!");
    bool blocking(false);
    bool compliant(false);
    driver_.lock();
    try {
        if (!driver_.move_trajectory_in_joints(goal->trajectory, blocking, compliant))
        {
            this->traj_in_joints_status_ = TRAJ_ERROR;
        } else {
            this->traj_in_joints_status_ = TRAJ_DONE;
        }
    } catch(CWamException &e) {
        ROS_INFO("%s", e.what().c_str());
    }
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
  // This sleep "assures" that the robot receives the trajectory and starts moving
  sleep(0.5f);
  driver_.lock();
  if (!driver_.is_moving())
  {
    ROS_INFO("[WamDriverNode]: FollowJointTrajectory FINISHED!");
    ret = true;
  } else {
    ROS_INFO("[WamDriverNode]: FollowJointTrajectory NOT FINISHED");
    ret = false;
  }
  driver_.unlock();
  return ret;
}

bool WamControllerDriverNode::follow_joint_trajectoryHasSucceedCallback(void)
{
  bool ret = false;
  // This sleep "assures" that the robot receives the trajectory and starts moving
  sleep(0.5f);
  driver_.lock();
  if (!driver_.is_moving() && (traj_in_joints_status_== TRAJ_DONE))
  {
    ROS_INFO("[WamDriverNode]: FollowJointTrajectory SUCCEEDED!");
    ret = true;
  } else {
    ROS_INFO("[WamDriverNode]: FollowJointTrajectory NOT SUCCEEDED");
    ret = false;
  }
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
