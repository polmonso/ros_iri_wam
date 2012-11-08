#include "wam_driver_node.h"

using namespace Eigen;

WamDriverNode::WamDriverNode(ros::NodeHandle &nh) :
 iri_base_driver::IriBaseNodeDriver<WamDriver>(nh),
  lwpr_trajectory_server_aserver_(public_node_handle_, "lwpr_trajectory"),
  joint_trajectory_aserver_(public_node_handle_, "joint_trajectory"),
// action_server_(nh,"iri_wam_pr2_controller/joint_trajectory_action",false),
 //action_server_follow_(nh,"iri_wam_pr2_controller/follow_joint_trajectory",false)
 action_server_(nh,"joint_trajectory_action",false),
 action_server_follow_(nh,"follow_joint_trajectory",false)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  //ask wam for the numaxes
  this->JointState_msg.name.resize(7);
  this->JointState_msg.position.resize(7); 

  // [init publishers]
  this->pose_publisher = this->public_node_handle_.advertise<geometry_msgs::PoseStamped>("pose", 5);
  this->joint_states_publisher = this->public_node_handle_.advertise<sensor_msgs::JointState>("joint_states", 5);

  // [init subscribers]

  // [init services]
  wam_services_server_ = public_node_handle_.advertiseService("wam_services",
                                                              &WamDriverNode::wam_servicesCallback, this);
  joints_move_server   = public_node_handle_.advertiseService("joints_move",
                                                              &WamDriverNode::joints_moveCallback, this);
  /* Pose movement in WAM is still not implemented.
   * Disable service publication.
   *
   * pose_move_server     = public_node_handle_.advertiseService("pose_move",
   *                                                           &WamDriverNode::pose_moveCallback, this);
   */

  // [init clients]

  // [init action servers]
  lwpr_trajectory_server_aserver_.registerStartCallback(boost::bind(&WamDriverNode::lwpr_trajectory_serverStartCallback, this, _1)); 
  lwpr_trajectory_server_aserver_.registerStopCallback(boost::bind(&WamDriverNode::lwpr_trajectory_serverStopCallback, this)); 
  lwpr_trajectory_server_aserver_.registerIsFinishedCallback(boost::bind(&WamDriverNode::lwpr_trajectory_serverIsFinishedCallback, this)); 
  lwpr_trajectory_server_aserver_.registerHasSucceedCallback(boost::bind(&WamDriverNode::lwpr_trajectory_serverHasSucceedCallback, this)); 
  lwpr_trajectory_server_aserver_.registerGetResultCallback(boost::bind(&WamDriverNode::lwpr_trajectory_serverGetResultCallback, this, _1)); 
  lwpr_trajectory_server_aserver_.registerGetFeedbackCallback(boost::bind(&WamDriverNode::lwpr_trajectory_serverGetFeedbackCallback, this, _1)); 
  lwpr_trajectory_server_aserver_.start();

  joint_trajectory_aserver_.registerStartCallback(boost::bind(&WamDriverNode::joint_trajectoryStartCallback, this, _1));
  joint_trajectory_aserver_.registerStopCallback(boost::bind(&WamDriverNode::joint_trajectoryStopCallback, this)); 
  joint_trajectory_aserver_.registerIsFinishedCallback(boost::bind(&WamDriverNode::joint_trajectoryIsFinishedCallback, this)); 
  joint_trajectory_aserver_.registerHasSucceedCallback(boost::bind(&WamDriverNode::joint_trajectoryHasSucceedCallback, this)); 
  joint_trajectory_aserver_.registerGetResultCallback(boost::bind(&WamDriverNode::joint_trajectoryGetResultCallback, this, _1)); 
  joint_trajectory_aserver_.registerGetFeedbackCallback(boost::bind(&WamDriverNode::joint_trajectoryGetFeedbackCallback, this, _1)); 
  joint_trajectory_aserver_.start();

  action_server_.registerGoalCallback(boost::bind(&WamDriverNode::goalCB, this, _1));
  //action_server_.registerCancelCallback(boost::bind(&WamDriverNode::cancelCB, this, _1));
  action_server_.start();
  action_server_follow_.registerGoalCallback(boost::bind(&WamDriverNode::goalFollowCB, this, _1));
  //action_server_.registerCancelCallback(boost::bind(&WamDriverNode::cancelFollowCB, this, _1));
  action_server_follow_.start();

  // [init action clients]

  ROS_INFO("Wam node started"); 
}

void WamDriverNode::mainNodeThread(void)
{
  //lock access to driver if necessary
  //this->driver_.lock();

  std::vector<double> angles(7,0.1);
  std::vector<double> pose(16,0);
  char jname[9];
  Matrix3f rmat;

  // [fill msg Header if necessary]
  this->PoseStamped_msg.header.stamp = ros::Time::now();
  this->PoseStamped_msg.header.frame_id = "wam_fk/wam7";

  // [fill msg structures]
  this->driver_.lock();
  this->driver_.get_pose(&pose);
  this->driver_.get_joint_angles(&angles);
  this->driver_.unlock();

  rmat << pose.at(0), pose.at(1),pose.at(2),pose.at(4),pose.at(5),pose.at(6),pose.at(8),pose.at(9),pose.at(10);

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
      snprintf(jname, 9, "j%d_joint", i+1);
      JointState_msg.name[i] = jname;
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

  if (! this->driver_.isRunning())
  {
    ROS_ERROR("Driver is not running");
    //unlock driver if previously blocked 
    this->driver_.unlock();
    return false;
  }

  //this call blocks if the wam faults. The mutex is not freed...!   
  //unlock driver if previously blocked 
  this->driver_.move_in_joints(& req.joints); 
  this->driver_.unlock();
  this->driver_.wait_move_end();

  return true;
}

bool WamDriverNode::pose_moveCallback(iri_wam_common_msgs::pose_move::Request &req, iri_wam_common_msgs::pose_move::Response &res) 
{ 
  bool result;
  //lock access to driver if necessary 
  this->driver_.lock();
  if(this->driver_.isRunning()){

     //do operations with req and output on res 
     //res.data2 = req.data1 + my_var; 
    Quaternion<float> quat(req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z);
    Matrix3f mat = quat.toRotationMatrix();

     std::vector <double> pose(16,0);
      ROS_INFO("Received Quat: %f %f %f %f %f %f %f\n",
                req.pose.position.x, req.pose.position.y, req.pose.position.z,
                req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w);
     pose[3] = req.pose.position.x;
     pose[7] = req.pose.position.y;
     pose[11] = req.pose.position.z;
     pose[15] = 1;
     for(int i=0; i<12; i++){
      if(i%4 != 3)
        pose[i] = mat(i/4,i%4);
     }
      ROS_INFO("Received Pose:\n %f %f %f %f \n %f %f %f %f \n %f %f %f %f\n %f %f %f %f\n",
            pose[0],pose[1],pose[2],pose[3],
            pose[4],pose[5],pose[6],pose[7],
            pose[8],pose[9],pose[10],pose[11],
            pose[12],pose[13],pose[14],pose[15]);

     this->driver_.move_in_cartesian(&pose);
     this->driver_.unlock();
     //do we want a blocking service?
     this->driver_.wait_move_end();
      result = true;

  }else{
    ROS_ERROR("Driver is not running at the moment");
    result = false;
    this->driver_.unlock();
  }

  res.success = result;
  return result;
}

/*  [action callbacks] */
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
    driver_.lock();
    driver_.move_trajectory_in_joints(goal->trajectory);
    driver_.unlock();
}

void
WamDriverNode::joint_trajectoryStopCallback(void) 
{
  driver_.lock();
  //stop action 
  driver_.unlock();
}

bool
WamDriverNode::joint_trajectoryIsFinishedCallback(void) 
{
    bool ret = false;

    driver_.lock();
    if (! driver_.is_moving())
        ret = true;
    driver_.unlock();

    return ret;
}

bool WamDriverNode::joint_trajectoryHasSucceedCallback(void) 
{ 
  bool ret = false; 

  driver_.lock();
    //if goal was accomplished 
    //ret = true 
  driver_.unlock(); 

  return ret; 
} 

void
WamDriverNode::joint_trajectoryGetResultCallback(control_msgs::FollowJointTrajectoryResultPtr& result) 
{
    // MSG has an empty result message
}

void WamDriverNode::joint_trajectoryGetFeedbackCallback(control_msgs::FollowJointTrajectoryFeedbackPtr& feedback) 
{ 
  driver_.lock(); 
    //keep track of feedback 
    //ROS_INFO("feedback: %s", feedback->data.c_str()); 
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
	    ii=traj.points.size()-1;
            this->driver_.move_in_joints(&traj.points[ii].positions); //this call blocks if the wam faults. The mutex is not freed...!
            this->driver_.unlock();
            this->driver_.wait_move_end();
        }
        else
        {
		    ROS_FATAL("Driver is not running");
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
