#include "wam_driver_node.h"

using namespace Eigen;

WamDriverNode::WamDriverNode(ros::NodeHandle &nh) :
 iri_base_driver::IriBaseNodeDriver<WamDriver>(nh),
 lwpr_trajectory_server_aserver_(public_node_handle_, "lwpr_trajectory"),
 follow_joint_trajectory_server_(public_node_handle_, "follow_joint_trajectory")
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  //ask wam for the numaxes
  this->JointState_msg.name.resize(7);
  this->JointState_msg.position.resize(7); 

  // [init publishers]
  this->pose_publisher = this->public_node_handle_.advertise<geometry_msgs::PoseStamped>("pose", 1);
  this->joint_states_publisher = this->public_node_handle_.advertise<sensor_msgs::JointState>("joint_states", 1);
  this->current_robot_state_publisher_ = this->public_node_handle_.advertise<moveit_msgs::DisplayRobotState>("robot_state", 1);

  // [init subscribers]

  // [init services]
  wam_services_server_ = public_node_handle_.advertiseService("wam_services",
                                                              &WamDriverNode::wam_servicesCallback, this);
  joints_move_server   = public_node_handle_.advertiseService("joints_move",
                                                              &WamDriverNode::joints_moveCallback, this);
  pose_move_server     = public_node_handle_.advertiseService("pose_move",
                                                              &WamDriverNode::pose_moveCallback, this);
  // [init clients]

  // [init action servers]
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
  this->PoseStamped_msg.header.stamp = ros::Time::now();
  this->PoseStamped_msg.header.frame_id = "wam_fk/wam7";

  // [fill msg structures]
  //this->DisplayRobotState_msg_.data = my_var;
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
      //char jname[9];
      //snprintf(jname, 9, "j%d_joint", i+1);
      char jname[12];
      snprintf(jname, 12, "wam_joint_%d", i+1);
      JointState_msg.name[i] = jname;
      JointState_msg.position[i] = angles[i];
  }

//  /* Load the robot model */
//  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//
//  /* Get a shared pointer to the model */
//  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//
//  /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
//  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
//
//  /* Get the configuration for the joints in the right arm of the PR2*/
//  robot_state::JointStateGroup* joint_state_group = kinematic_state->getJointStateGroup("arm1");


  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->current_robot_state_publisher_.publish(this->DisplayRobotState_msg_);
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
        ROS_ERROR("Driver is not running at the moment");
        result = false;
    }
    this->driver_.unlock();

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
