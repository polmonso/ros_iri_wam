#include "wam_driver_node.h"

using namespace Eigen;

WamDriverNode::WamDriverNode(ros::NodeHandle &nh) :
 iri_base_driver::IriBaseNodeDriver<WamDriver>(nh),
 DMPTracker_aserver_(public_node_handle_, "DMPTracker"),
 lwpr_trajectory_server_aserver_(public_node_handle_, "lwpr_trajectory"),
 follow_joint_trajectory_server_(public_node_handle_, "follow_joint_trajectory")
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  //ask wam for the numaxes
  this->JointState_msg.name.resize(7);
  this->JointState_msg.position.resize(7); 

  // [init publishers]
  this->pose_publisher = this->public_node_handle_.advertise<geometry_msgs::PoseStamped>("libbarrett_link_tcp", 1);
  this->joint_states_publisher = this->public_node_handle_.advertise<sensor_msgs::JointState>("/joint_states", 1);

  // [init subscribers]
  this->jnt_pos_cmd_subscriber_ = this->public_node_handle_.subscribe("jnt_pos_cmd", 1, &WamDriverNode::jnt_pos_cmd_callback, this);
  this->DMPTrackerNewGoal_subscriber_ = this->public_node_handle_.subscribe("DMPTrackerNewGoal", 1, &WamDriverNode::DMPTrackerNewGoal_callback, this);

  // [init services]
  wam_services_server_ = public_node_handle_.advertiseService("wam_services",
                                                              &WamDriverNode::wam_servicesCallback, this);
  joints_move_server   = public_node_handle_.advertiseService("joints_move",
                                                              &WamDriverNode::joints_moveCallback, this);
  pose_move_server     = public_node_handle_.advertiseService("pose_move",
                                                              &WamDriverNode::pose_moveCallback, this);
  // [init clients]

  // [init action servers]
  DMPTracker_aserver_.registerStartCallback(boost::bind(&WamDriverNode::DMPTrackerStartCallback, this, _1)); 
  DMPTracker_aserver_.registerStopCallback(boost::bind(&WamDriverNode::DMPTrackerStopCallback, this)); 
  DMPTracker_aserver_.registerIsFinishedCallback(boost::bind(&WamDriverNode::DMPTrackerIsFinishedCallback, this)); 
  DMPTracker_aserver_.registerHasSucceedCallback(boost::bind(&WamDriverNode::DMPTrackerHasSucceedCallback, this)); 
  DMPTracker_aserver_.registerGetResultCallback(boost::bind(&WamDriverNode::DMPTrackerGetResultCallback, this, _1)); 
  DMPTracker_aserver_.registerGetFeedbackCallback(boost::bind(&WamDriverNode::DMPTrackerGetFeedbackCallback, this, _1)); 
  DMPTracker_aserver_.start();
  
  lwpr_trajectory_server_aserver_.registerStartCallback(boost::bind(&WamDriverNode::lwpr_trajectory_serverStartCallback, this, _1)); 
  lwpr_trajectory_server_aserver_.registerStopCallback(boost::bind(&WamDriverNode::lwpr_trajectory_serverStopCallback, this)); 
  lwpr_trajectory_server_aserver_.registerIsFinishedCallback(boost::bind(&WamDriverNode::lwpr_trajectory_serverIsFinishedCallback, this)); 
  lwpr_trajectory_server_aserver_.registerHasSucceedCallback(boost::bind(&WamDriverNode::lwpr_trajectory_serverHasSucceedCallback, this)); 
  lwpr_trajectory_server_aserver_.registerGetResultCallback(boost::bind(&WamDriverNode::lwpr_trajectory_serverGetResultCallback, this, _1)); 
  lwpr_trajectory_server_aserver_.registerGetFeedbackCallback(boost::bind(&WamDriverNode::lwpr_trajectory_serverGetFeedbackCallback, this, _1)); 
  lwpr_trajectory_server_aserver_.start();

  follow_joint_trajectory_server_.registerStartCallback(boost::bind(&WamDriverNode::joint_trajectoryStartCallback, this, _1));
  follow_joint_trajectory_server_.registerStopCallback(boost::bind(&WamDriverNode::joint_trajectoryStopCallback, this)); 
  follow_joint_trajectory_server_.registerIsFinishedCallback(boost::bind(&WamDriverNode::joint_trajectoryIsFinishedCallback, this)); 
  follow_joint_trajectory_server_.registerHasSucceedCallback(boost::bind(&WamDriverNode::joint_trajectoryHasSucceedCallback, this)); 
  follow_joint_trajectory_server_.registerGetResultCallback(boost::bind(&WamDriverNode::joint_trajectoryGetResultCallback, this, _1)); 
  follow_joint_trajectory_server_.registerGetFeedbackCallback(boost::bind(&WamDriverNode::joint_trajectoryGetFeedbackCallback, this, _1)); 
  follow_joint_trajectory_server_.start();

  // [init action clients]

  ROS_INFO("Wam node started"); 
}

void WamDriverNode::mainNodeThread(void)
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

  this->PoseStamped_msg.header.stamp = ros::Time::now();
  this->PoseStamped_msg.header.frame_id = ss_tcp_link_name.str().c_str();

  // [fill msg structures]
  this->driver_.lock();
  this->driver_.get_pose(&pose);
  this->driver_.get_joint_angles(&angles);
  this->driver_.unlock();

  rmat << pose.at(0), pose.at(1), pose.at(2), pose.at(4), pose.at(5), pose.at(6), pose.at(8), pose.at(9), pose.at(10);

  {
    Quaternion<float> quat(rmat);

    this->PoseStamped_msg.pose.position.x = pose.at(3);
    this->PoseStamped_msg.pose.position.y = pose.at(7);
    this->PoseStamped_msg.pose.position.z = pose.at(11);
    this->PoseStamped_msg.pose.orientation.x = quat.x();
    this->PoseStamped_msg.pose.orientation.y = quat.y();
    this->PoseStamped_msg.pose.orientation.z = quat.z();
    this->PoseStamped_msg.pose.orientation.w = quat.w();
  }

  JointState_msg.header.stamp = ros::Time::now();
  for(int i=0;i<(int)angles.size();i++){
      std::stringstream ss_jname;
      ss_jname << robot_name << "_joint_" << i+1;
      JointState_msg.name[i] = ss_jname.str().c_str();
      JointState_msg.position[i] = angles[i];
  }

  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->joint_states_publisher.publish(this->JointState_msg);
  this->pose_publisher.publish(this->PoseStamped_msg);

  //unlock access to driver if previously blocked
  //this->driver_.unlock();
}

/*  [subscriber callbacks] */
void WamDriverNode::jnt_pos_cmd_callback(const wam_msgs::RTJointPos::ConstPtr& msg) 
{ 
  ROS_INFO("WamDriverNode::jnt_pos_cmd_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->driver_.lock(); 
  //this->jnt_pos_cmd_mutex_.enter(); 

  //std::cout << msg->data << std::endl; 

  driver_.jnt_pos_cmd_callback(&msg->joints,&msg->rate_limits);
  //unlock previously blocked shared variables 
  //this->driver_.unlock(); 
  //this->jnt_pos_cmd_mutex_.exit(); 
}
void WamDriverNode::DMPTrackerNewGoal_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg) 
{ 
  ROS_INFO("WamDriverNode::DMPTrackerNewGoal_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->driver_.lock(); 
  //this->DMPTrackerNewGoal_mutex_.enter(); 
  driver_.dmp_tracker_new_goal(&msg->positions);
  //std::cout << msg->data << std::endl; 

  //unlock previously blocked shared variables 
  //this->driver_.unlock(); 
  //this->DMPTrackerNewGoal_mutex_.exit(); 
}

/*  [service callbacks] */
bool WamDriverNode::wam_servicesCallback(iri_wam_common_msgs::wamdriver::Request &req, iri_wam_common_msgs::wamdriver::Response &res) 
{ 
  //lock access to driver if necessary 
  this->driver_.lock(); 

  if(this->driver_.isRunning()) 
  { 
    switch(req.call){
      case HOLDON:
        this->driver_.hold_current_position(true);
        break;
      case HOLDOFF:
        this->driver_.hold_current_position(false);
        break;
      default:
          ROS_ERROR("Invalid action id. Check wam_actions_node.h");
        break;
    }
    //do operations with req and output on res 
    //res.data2 = req.data1 + my_var; 
  } else { 
    std::cout << "ERROR: Driver is not on run mode yet." << std::endl; 
  } 

  //unlock driver if previously blocked 
  this->driver_.unlock(); 

  return true; 
}
bool WamDriverNode::joints_moveCallback(iri_wam_common_msgs::joints_move::Request  & req,
                                        iri_wam_common_msgs::joints_move::Response & res)
{
  //lock access to driver if necessary
  this->driver_.lock();

  if (!this->driver_.isRunning())
  {
    ROS_ERROR("[JointsMoveCB] Driver is not running");
    //unlock driver if previously blocked 
    this->driver_.unlock();
    return false;
  }

  //this call blocks if the wam faults. The mutex is not freed...!   
  //unlock driver if previously blocked 
  if ((req.velocities.size() == 0) || (req.accelerations.size() == 0))
    this->driver_.move_in_joints(& req.joints); 
  else
    this->driver_.move_in_joints(& req.joints, &req.velocities, &req.accelerations); 
  this->driver_.unlock();
  this->driver_.wait_move_end();

  return true;
}

bool
WamDriverNode::pose_moveCallback(iri_wam_common_msgs::pose_move::Request  & req,
                                 iri_wam_common_msgs::pose_move::Response & res)
{
    bool result;

    //lock access to driver if necessary 
    this->driver_.lock();
    if (this->driver_.isRunning())
    {

        ROS_DEBUG("Received cartesian pose for movement: %f %f %f %f %f %f %f\n",
                req.pose.position.x, req.pose.position.y, req.pose.position.z,
                req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w);

        this->driver_.move_in_cartesian_pose(req.pose, req.vel, req.acc);
        this->driver_.wait_move_end();
        result = true;
    }
    else
    {
        ROS_ERROR("[PoseMoveCB] Driver is not running");
        result = false;
    }
    this->driver_.unlock();

    res.success = result;
    return result;
}

/*  [action callbacks] */
void WamDriverNode::DMPTrackerStartCallback(const iri_wam_common_msgs::DMPTrackerGoalConstPtr& goal)
{ 
  driver_.lock(); 
    //check goal 
    //execute goal 
  driver_.start_dmp_tracker(&goal->initial.positions,&goal->goal.positions);
  driver_.unlock(); 
} 

void WamDriverNode::DMPTrackerStopCallback(void) 
{ 
  driver_.lock(); 
    //stop action 
  driver_.unlock(); 
} 

bool WamDriverNode::DMPTrackerIsFinishedCallback(void) 
{ 
  bool ret = false; 

  driver_.lock(); 
    //if action has finish for any reason 
    //ret = true; 
  driver_.unlock(); 

  return ret; 
} 

bool WamDriverNode::DMPTrackerHasSucceedCallback(void) 
{ 
  bool ret = false; 

  driver_.lock(); 
    //if goal was accomplished 
    //ret = true; 
  driver_.unlock(); 

  return ret; 
} 

void WamDriverNode::DMPTrackerGetResultCallback(iri_wam_common_msgs::DMPTrackerResultPtr& result) 
{ 
  driver_.lock(); 
    //update result data to be sent to client 
    //result->data = data; 
  driver_.unlock(); 
} 

void WamDriverNode::DMPTrackerGetFeedbackCallback(iri_wam_common_msgs::DMPTrackerFeedbackPtr& feedback) 
{ 
  driver_.lock(); 
    //keep track of feedback 
    //ROS_INFO("feedback: %s", feedback->data.c_str()); 
  driver_.unlock(); 
}
void WamDriverNode::lwpr_trajectory_serverStartCallback(const iri_wam_common_msgs::LWPRTrajectoryReturningForceEstimationGoalConstPtr& goal)
{
    driver_.lock();
    driver_.move_trajectory_learnt_and_estimate_force(goal->model_filename, goal->points_filename);
    driver_.unlock();
}

void WamDriverNode::lwpr_trajectory_serverStopCallback(void) 
{
    driver_.lock();
    // TODO: not implemented yet
    driver_.unlock();
}

bool WamDriverNode::lwpr_trajectory_serverIsFinishedCallback(void) 
{
    bool ret = false;

    driver_.lock();
    if (driver_.get_force_request_info()->is_estimate_force_request_finish())
         ret = true;
    driver_.unlock();

    return ret;
}

bool WamDriverNode::lwpr_trajectory_serverHasSucceedCallback(void) 
{
    bool ret = false;

    driver_.lock();
    ret = driver_.get_force_request_info()->was_estimate_force_request_succedded();
    driver_.unlock();

    return ret;
}

void WamDriverNode::lwpr_trajectory_serverGetResultCallback(iri_wam_common_msgs::LWPRTrajectoryReturningForceEstimationResultPtr& result)
{
    driver_.lock();
    result->force = driver_.get_force_request_info()->force_value;
    driver_.unlock();
}

void WamDriverNode::lwpr_trajectory_serverGetFeedbackCallback(iri_wam_common_msgs::LWPRTrajectoryReturningForceEstimationFeedbackPtr& feedback) 
{
    // TODO: not feedback provided at the moment
    driver_.lock();
   //keep track of feedback 
    //ROS_INFO("feedback: %s", feedback->data.c_str()); 
    driver_.unlock(); 
}
void
WamDriverNode::joint_trajectoryStartCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
    // Need to get the list of positions and send it to the driver
    ROS_INFO("[WamDriverNode]: New FollowJointTrajectoryGoal RECEIVED!"); 
    driver_.lock();
    driver_.move_trajectory_in_joints(goal->trajectory);
    driver_.unlock();
    ROS_INFO("[WamDriverNode]: FollowJointTrajectoryGoal SENT TO ROBOT!"); 
}

void
WamDriverNode::joint_trajectoryStopCallback(void) 
{
    ROS_INFO("[WamDriverNode]: New CancelFollowJointTrajectoryAction RECEIVED!"); 
    driver_.lock();
    driver_.stop_trajectory_in_joints();
    driver_.unlock();
    ROS_INFO("[WamDriverNode]: CancelFollowJointTrajectoryAction SENT TO ROBOT!"); 
}

bool
WamDriverNode::joint_trajectoryIsFinishedCallback(void) 
{
    bool ret = false;
    ROS_INFO("[WamDriverNode]: FollowJointTrajectory NOT FINISHED"); 

    // This sleep assures that the robot receives the trajectory and starts moving
    sleep(1);

    driver_.lock();
    if (! driver_.is_moving()){
        ret = true;
        ROS_INFO("[WamDriverNode]: FollowJointTrajectory FINISHED!"); 
    }
    driver_.unlock();

    return ret;
}

bool WamDriverNode::joint_trajectoryHasSucceedCallback(void) 
{ 
  bool ret = false; 

  driver_.lock();
  //This callback is called when the trajectory is Finished so by
  // default is always true
  ret = true;
  driver_.unlock(); 

  ROS_INFO("[WamDriverNode]: FollowJointTrajectory SUCCEEDED!");

  return ret; 
} 

void
WamDriverNode::joint_trajectoryGetResultCallback(control_msgs::FollowJointTrajectoryResultPtr& result) 
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

void WamDriverNode::joint_trajectoryGetFeedbackCallback(control_msgs::FollowJointTrajectoryFeedbackPtr& feedback) 
{ 
  driver_.lock(); 
    //keep track of feedback 
    //ROS_INFO("feedback: %s", feedback->data.c_str()); 
  feedback->desired = driver_.get_desired_joint_trajectory_point();

  driver_.unlock(); 
}
void WamDriverNode::goalCB(GoalHandle gh)
{
    gh.setAccepted();
    bool state=false;
    trajectory_msgs::JointTrajectory traj = gh.getGoal()->trajectory;
    trajectory2follow(traj,state);
    if(state) gh.setSucceeded();
    else gh.setAborted();
}
void WamDriverNode::goalFollowCB(GoalHandleFollow gh)
{
    gh.setAccepted();
    bool state=false;
    trajectory_msgs::JointTrajectory traj = gh.getGoal()->trajectory;
    trajectory2follow(traj,state);
    if(state) gh.setSucceeded();
    else gh.setAborted();
}
void WamDriverNode::trajectory2follow(trajectory_msgs::JointTrajectory traj, bool& state)
{
	this->driver_.lock();
	for(unsigned int ii=0; ii < traj.points.size(); ++ii)
	{
        if(this->driver_.isRunning())
        {
            //ii=traj.points.size()-1;
            this->driver_.move_in_joints(&traj.points[ii].positions); //this call blocks if the wam faults. The mutex is not freed...!
            this->driver_.unlock();
            this->driver_.wait_move_end();
        }
        else
        {
		    ROS_FATAL("[Trajectory2follow] Driver is not running");
		    state=false;
	    }
    }
   state=true;	
} 



/*  [action requests] */

void WamDriverNode::postNodeOpenHook(void)
{
}

void WamDriverNode::addNodeDiagnostics(void)
{
}

void WamDriverNode::addNodeOpenedTests(void)
{
}

void WamDriverNode::addNodeStoppedTests(void)
{
}

void WamDriverNode::addNodeRunningTests(void)
{
}

void WamDriverNode::reconfigureNodeHook(int level)
{
}

WamDriverNode::~WamDriverNode()
{
  //[free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<WamDriverNode>(argc,argv,"wam_driver_node");
}
