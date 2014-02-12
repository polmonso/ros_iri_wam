#include "wam_dmp_tracker_driver_node.h"

WamDmpTrackerDriverNode::WamDmpTrackerDriverNode(ros::NodeHandle &nh) : 
  iri_base_driver::IriBaseNodeDriver<WamDmpTrackerDriver>(nh),
  DMPTracker_client_("/iri_wam/iri_wam_controller/dmp_joint_tracker", true)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->DMPTrackerNewGoal_publisher_ = this->public_node_handle_.advertise<trajectory_msgs::JointTrajectoryPoint>("DMPTracker_output", 1);
  
  // [init subscribers]
  this->pose_surface_subscriber_ = this->public_node_handle_.subscribe("pose_surface", 1, &WamDmpTrackerDriverNode::pose_surface_callback, this);
  
  // [init services]
  
  // [init clients]
  wamik_client_ = this->public_node_handle_.serviceClient<iri_wam_common_msgs::wamInverseKinematics>("DMP_ik");
  
  // [init action servers]
  
  // [init action clients]

//Init the tracker and the robot  
    DMPTrackerMakeActionRequest();

  
}

void WamDmpTrackerDriverNode::mainNodeThread(void)
{
  //lock access to driver if necessary
  this->driver_.lock();

  // [fill msg Header if necessary]
  //<publisher_name>.header.stamp = ros::Time::now();
  //<publisher_name>.header.frame_id = "<publisher_topic_name>";

  // [fill msg structures]
  //this->JointTrajectoryPoint_msg_.data = my_var;
  
  // [fill srv structure and make request to the server]

  
  // [fill action structure and make request to the action server]
 // DMPTrackerMakeActionRequest();

  // [publish messages]

  //unlock access to driver if previously blocked
  this->driver_.unlock();
}

/*  [subscriber callbacks] */
void WamDmpTrackerDriverNode::pose_surface_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{ 
  ROS_INFO("WamDmpTrackerDriverNode::pose_surface_callback: New Message Received"); 
  /*
  tf::TransformListener listener; 
  tf::StampedTransform tcp_H_wam7;
  tf::StampedTransform tcp_H_wam7;
  
  ROS_INFO("WamDmpTrackerDriverNode::pose_surface_callback: New Message Received"); 
  
  try{
    ros::Time now = ros::Time::now();
    ros::Duration interval = ros::Duration(1.0);
    if(!listener.waitForTransform("camera_depth_optical_frame", "/estirabot_link_base", now, interval)){
        ROS_ERROR("Timeout while waiting for transform between frames camera_depth_optical_frame and /wam_tcp/ "); 
    }
    listener.lookupTransform("camera_depth_optical_frame", "/estirabot_link_base", now, tcp_H_wam7);
  }catch (tf::TransformException ex){
    ROS_ERROR("lookup transform error: %s", ex.what());
    return false;
  }
  
  
  tcp_H_wam7
  */
  //[fill srv structure and make request to the server]
  wamik_srv_.request.pose.header.frame_id = msg->header.frame_id;
  wamik_srv_.request.pose.pose = msg->pose;
  wamik_srv_.request.pose.pose.orientation.x =  0.001070f;
  wamik_srv_.request.pose.pose.orientation.y =  0.840995f;
  wamik_srv_.request.pose.pose.orientation.z =  0.000534f;
  wamik_srv_.request.pose.pose.orientation.w =  0.541041f;
  ROS_INFO("WamDmpTrackerDriverNode:: Response: %f %f %f %f %f %f %f", 
	                                               wamik_srv_.request.pose.pose.position.x,
                                                       wamik_srv_.request.pose.pose.position.y,
                                                       wamik_srv_.request.pose.pose.position.z,
                                                       wamik_srv_.request.pose.pose.orientation.x,
                                                       wamik_srv_.request.pose.pose.orientation.y,
                                                       wamik_srv_.request.pose.pose.orientation.z,
                                                       wamik_srv_.request.pose.pose.orientation.w
	  );
  ROS_INFO("WamDmpTrackerDriverNode:: Sending New Request!"); 
  if (wamik_client_.call(wamik_srv_)) 
  { 
    ROS_INFO("WamDmpTrackerDriverNode:: Response: %f %f %f %f %f %f %f", 
	                                               wamik_srv_.response.joints.position[0],
                                                       wamik_srv_.response.joints.position[1],
                                                       wamik_srv_.response.joints.position[2],
                                                       wamik_srv_.response.joints.position[3],
                                                       wamik_srv_.response.joints.position[4],
                                                       wamik_srv_.response.joints.position[5],
                                                       wamik_srv_.response.joints.position[6]
    ); 
    
    //JointTrajectoryPoint_msg_.positions[0]
//    this->DMPTrackerNewGoal_publisher_.publish(this->JointTrajectoryPoint_msg_);
    trajectory_msgs::JointTrajectoryPoint new_joints;
    new_joints.positions.resize(7);
    for (size_t i=0;i<7;i++) 
      new_joints.positions[i] = wamik_srv_.response.joints.position[i];
    //this->DMPTrackerNewGoal_publisher_.publish(wamik_srv_.response.joints);
    this->DMPTrackerNewGoal_publisher_.publish(new_joints);

  } 
  else 
  { 
    ROS_INFO("WamDmpTrackerDriverNode:: Failed to Call Server on topic wamik "); 
  }
  
  
  
  //use appropiate mutex to shared variables if necessary 
  //this->driver_.lock(); 
  //this->pose_surface_mutex_.enter(); 

  
  
  //std::cout << msg->data << std::endl; 

  //unlock previously blocked shared variables 
  //this->driver_.unlock(); 
  //this->pose_surface_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */
void WamDmpTrackerDriverNode::DMPTrackerDone(const actionlib::SimpleClientGoalState& state,  const iri_wam_common_msgs::DMPTrackerResultConstPtr& result) 
{ 
  if( state.toString().compare("SUCCEEDED") == 0 ) 
    ROS_INFO("WamDmpTrackerDriverNode::DMPTrackerDone: Goal Achieved!"); 
  else 
    ROS_INFO("WamDmpTrackerDriverNode::DMPTrackerDone: %s", state.toString().c_str()); 

  //copy & work with requested result 
} 

void WamDmpTrackerDriverNode::DMPTrackerActive() 
{ 
  //ROS_INFO("WamDmpTrackerDriverNode::DMPTrackerActive: Goal just went active!"); 
} 

void WamDmpTrackerDriverNode::DMPTrackerFeedback(const iri_wam_common_msgs::DMPTrackerFeedbackConstPtr& feedback) 
{ 
  //ROS_INFO("WamDmpTrackerDriverNode::DMPTrackerFeedback: Got Feedback!"); 

  bool feedback_is_ok = true; 

  //analyze feedback 
  //my_var = feedback->var; 

  //if feedback is not what expected, cancel requested goal 
  if( !feedback_is_ok ) 
  { 
    DMPTracker_client_.cancelGoal(); 
    //ROS_INFO("WamDmpTrackerDriverNode::DMPTrackerFeedback: Cancelling Action!"); 
  } 
}

/*  [action requests] */
void WamDmpTrackerDriverNode::DMPTrackerMakeActionRequest() 
{ 
  ROS_INFO("WamDmpTrackerDriverNode::DMPTrackerMakeActionRequest: Starting New Request!"); 

  //wait for the action server to start 
  //will wait for infinite time 
  DMPTracker_client_.waitForServer();  
  ROS_INFO("WamDmpTrackerDriverNode::DMPTrackerMakeActionRequest: Server is Available!"); 

  //send a goal to the action 
  //   rostopic pub /pose_surface geometry_msgs/PoseStamped "{header: {stamp:ow, frame_id: "camera_depth_optical_frame"},pose: {position: {x: 0, y: -0.35, z: 0.49}, orientation: {x: -0.374, y: -0.374, z: -0.54, w: 0.65}}}" 
  //DMPTracker_goal_.data = my_desired_goal;
    DMPTracker_goal_.initial.positions.resize(7);   
    DMPTracker_goal_.initial.positions[0] = 0.0; 
    DMPTracker_goal_.initial.positions[1] = 0.0;
    DMPTracker_goal_.initial.positions[2] = 0.0;
    DMPTracker_goal_.initial.positions[3] = 2.1;
    DMPTracker_goal_.initial.positions[4] = 0.0;
    DMPTracker_goal_.initial.positions[5] = 0.0;
    DMPTracker_goal_.initial.positions[6] = 0.0;

    DMPTracker_goal_.goal.positions.resize(7); 
    DMPTracker_goal_.goal.positions[0] = 0.0; 
    DMPTracker_goal_.goal.positions[1] = 0.0;
    DMPTracker_goal_.goal.positions[2] = 0.0;
    DMPTracker_goal_.goal.positions[3] = 2.0;
    DMPTracker_goal_.goal.positions[4] = 0.0;
    DMPTracker_goal_.goal.positions[5] = 0.0;
    DMPTracker_goal_.goal.positions[6] = 0.0;
    
    //DMPTracker_goal_.initial.positions.resize(7);   
    //DMPTracker_goal_.initial.positions[0] = 0.851;
    //DMPTracker_goal_.initial.positions[1] = 1.33;
    //DMPTracker_goal_.initial.positions[2] = -1.23;
    //DMPTracker_goal_.initial.positions[3] = 2.18;
    //DMPTracker_goal_.initial.positions[4] = 0.24;
    //DMPTracker_goal_.initial.positions[5] = -1.37;
    //DMPTracker_goal_.initial.positions[6] = 1.23;

    //DMPTracker_goal_.goal.positions.resize(7); 
    //DMPTracker_goal_.goal.positions[0] = 0.54;
    //DMPTracker_goal_.goal.positions[1] = 1.56;
    //DMPTracker_goal_.goal.positions[2] = -1.43;
    //DMPTracker_goal_.goal.positions[3] = 1.63;
    //DMPTracker_goal_.goal.positions[4] = 0.14;
    //DMPTracker_goal_.goal.positions[5] = -1.16;
    //DMPTracker_goal_.goal.positions[6] = 1.53;
    
  DMPTracker_client_.sendGoal(DMPTracker_goal_, 
              boost::bind(&WamDmpTrackerDriverNode::DMPTrackerDone,     this, _1, _2), 
              boost::bind(&WamDmpTrackerDriverNode::DMPTrackerActive,   this), 
              boost::bind(&WamDmpTrackerDriverNode::DMPTrackerFeedback, this, _1)); 
  ROS_INFO("WamDmpTrackerDriverNode::DMPTrackerMakeActionRequest: Goal Sent. Wait for Result!"); 
}

void WamDmpTrackerDriverNode::postNodeOpenHook(void)
{
}

void WamDmpTrackerDriverNode::addNodeDiagnostics(void)
{
}

void WamDmpTrackerDriverNode::addNodeOpenedTests(void)
{
}

void WamDmpTrackerDriverNode::addNodeStoppedTests(void)
{
}

void WamDmpTrackerDriverNode::addNodeRunningTests(void)
{
}

void WamDmpTrackerDriverNode::reconfigureNodeHook(int level)
{
}

WamDmpTrackerDriverNode::~WamDmpTrackerDriverNode(void)
{
  // [free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<WamDmpTrackerDriverNode>(argc, argv, "wam_dmp_tracker_driver_node");
}
