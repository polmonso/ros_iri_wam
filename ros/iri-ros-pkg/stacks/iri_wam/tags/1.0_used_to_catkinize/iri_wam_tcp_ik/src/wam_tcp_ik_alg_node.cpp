#include "wam_tcp_ik_alg_node.h"

WamTcpIkAlgNode::WamTcpIkAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<WamTcpIkAlgorithm>()
{
  // get the frame from the parameter server
  public_node_handle_.param<std::string>("robot_base", robot_base_str_, "robot_base_frame");
  public_node_handle_.param<std::string>("robot_tcp", robot_tcp_str_, "robot_tcp_frame");
  public_node_handle_.param<std::string>("tool_tcp", tool_tcp_str_, "tool_tcp_frame");
  
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  get_ik_server_ = this->public_node_handle_.advertiseService( "get_wam_ik", &WamTcpIkAlgNode::get_ikCallback, this);
  get_robot_pose_server_ = this->public_node_handle_.advertiseService( "get_wam_robot_pose", &WamTcpIkAlgNode::get_robotPoseCallback, this);
  
  // [init clients]
  get_ik_client_ = this->public_node_handle_.serviceClient<iri_wam_common_msgs::wamInverseKinematics>("wamik");
  
  // [init action servers]
  
  // [init action clients]
}

WamTcpIkAlgNode::~WamTcpIkAlgNode(void)
{
  // [free dynamic memory]
}

void WamTcpIkAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  //get_ik_srv_.request.data = my_var; 
  //ROS_INFO("WamTcpIkAlgNode:: Sending New Request!"); 
  //if (get_ik_client_.call(get_ik_srv)) 
  //{ 
    //ROS_INFO("WamTcpIkAlgNode:: Response: %s", get_ik_srv_.response.result); 
  //} 
  //else 
  //{ 
    //ROS_INFO("WamTcpIkAlgNode:: Failed to Call Server on topic get_ik "); 
  //}
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */
bool WamTcpIkAlgNode::get_ikCallback(iri_wam_common_msgs::wamInverseKinematics::Request &req, iri_wam_common_msgs::wamInverseKinematics::Response &res) 
{ 
  bool result(false);

  ROS_INFO("WamTcpIkAlgNode::get_ikCallback: New Request Received!"); 
  
  //use appropiate mutex to shared variables if necessary 
  this->alg_.lock(); 
  //this->get_ik_mutex_.enter(); 
  
  // PREDEFINED_TCP TO WAM_TCP
  try{
    ros::Time now = ros::Time::now();
    ros::Duration interval = ros::Duration(1.0);
    if(!listener_.waitForTransform(tool_tcp_str_, robot_tcp_str_, now, interval)){
        ROS_ERROR("Timeout while waiting for transform between frames %s and %s ", tool_tcp_str_.c_str(), robot_tcp_str_.c_str()); 
    }
    listener_.lookupTransform(tool_tcp_str_, robot_tcp_str_, now, tcp_H_wam7_);
  }catch (tf::TransformException ex){
    ROS_ERROR("lookup transform error: %s", ex.what());
    return false;
  }

  // Pose from robot_tcp To user_tcp
  ROS_INFO("[WamTcpIkAlgNode] %s_H_7 Pose (x, y, z, qx, qy, qz, qw): [ %f, %f, %f, %f, %f, %f, %f ]",
            tool_tcp_str_.c_str(),
            tcp_H_wam7_.getOrigin().x(),
            tcp_H_wam7_.getOrigin().y(),
            tcp_H_wam7_.getOrigin().z(),
            tcp_H_wam7_.getRotation().x(), 
            tcp_H_wam7_.getRotation().y(), 
            tcp_H_wam7_.getRotation().z(), 
            tcp_H_wam7_.getRotation().w());

  // Destination cartesian pose in World coordinates 
  ROS_INFO("[WamTcpIkAlgNode] Received Pose from frame %s (x, y, z, qx, qy, qz, qw): [ %f, %f, %f, %f, %f, %f, %f ]",
            req.pose.header.frame_id.c_str(),
            req.pose.pose.position.x,
            req.pose.pose.position.y, 
            req.pose.pose.position.z,
            req.pose.pose.orientation.x,
            req.pose.pose.orientation.y,
            req.pose.pose.orientation.z,
            req.pose.pose.orientation.w);

  // User frame_id pose
  tf::Quaternion world_quat_tcp( req.pose.pose.orientation.x, req.pose.pose.orientation.y, req.pose.pose.orientation.z, req.pose.pose.orientation.w);
  tf::Vector3 world_pos_tcp( req.pose.pose.position.x, req.pose.pose.position.y, req.pose.pose.position.z);
  tf::Transform received_pose( world_quat_tcp, world_pos_tcp);

  // TF from world(/iri_wam_link_base) 
  // Usually, this transformation will be the identity.
  // Because the frame_id of the requested pose will usually be "/iri_wam_link_base".
  // But sometimes the user may ask for a pose that is referenced from another
  // frame.
  try{
    ros::Time now = ros::Time::now();
    ros::Duration interval = ros::Duration(1.0);
    if(!listener_.waitForTransform(robot_base_str_, req.pose.header.frame_id, now, interval)){
        ROS_ERROR("Timeout while waiting for transform between frames %s and %s ", robot_base_str_.c_str(), req.pose.header.frame_id.c_str()); 
    }
    listener_.lookupTransform(robot_base_str_, req.pose.header.frame_id, now, world_H_wam7_);
  }catch (tf::TransformException ex){
    ROS_ERROR("lookup transform error: %s", ex.what());
    return false;
  }
 
  world_H_wam7_ *= received_pose;
  world_H_wam7_ *= tcp_H_wam7_;
  
  base_pose_msg_.request.pose.header.frame_id    = robot_base_str_; 
  base_pose_msg_.request.pose.pose.position.x    = world_H_wam7_.getOrigin().x(); 
  base_pose_msg_.request.pose.pose.position.y    = world_H_wam7_.getOrigin().y(); 
  base_pose_msg_.request.pose.pose.position.z    = world_H_wam7_.getOrigin().z(); 
  base_pose_msg_.request.pose.pose.orientation.x = world_H_wam7_.getRotation().x(); 
  base_pose_msg_.request.pose.pose.orientation.y = world_H_wam7_.getRotation().y(); 
  base_pose_msg_.request.pose.pose.orientation.z = world_H_wam7_.getRotation().z(); 
  base_pose_msg_.request.pose.pose.orientation.w = world_H_wam7_.getRotation().w(); 

  ROS_INFO("[WamTcpIkAlgNode] 0_H_7 Pose (x, y, z, qx, qy, qz, qw): [ %f, %f, %f, %f, %f, %f, %f ]",
           world_H_wam7_.getOrigin().x(),
           world_H_wam7_.getOrigin().y(),
           world_H_wam7_.getOrigin().z(),
           world_H_wam7_.getRotation().x(), 
           world_H_wam7_.getRotation().y(), 
           world_H_wam7_.getRotation().z(), 
           world_H_wam7_.getRotation().w());

  if(!get_ik_client_.call(base_pose_msg_)){
    ROS_ERROR("Failed to call service %s", get_ik_client_.getService().c_str());
    result = false;
  }
  
  res.joints.position.resize(7);
  for(int ii=0; ii < 7; ++ii)
  {
      res.joints.position[ii] = base_pose_msg_.response.joints.position[ii];
  }
  ROS_INFO("[WamTcpIkAlgNode] Joints readings: (%f, %f, %f, %f, %f, %f, %f)",
          base_pose_msg_.response.joints.position[0], 
          base_pose_msg_.response.joints.position[1], 
          base_pose_msg_.response.joints.position[2], 
          base_pose_msg_.response.joints.position[3], 
          base_pose_msg_.response.joints.position[4],
          base_pose_msg_.response.joints.position[5], 
          base_pose_msg_.response.joints.position[6] );
  result = true;

  //unlock previously blocked shared variables 
  this->alg_.unlock(); 
  //this->get_ik_mutex_.exit(); 

  return result; 
}

bool WamTcpIkAlgNode::get_robotPoseCallback(iri_wam_common_msgs::wamGetRobotPoseFromToolPose::Request &req, iri_wam_common_msgs::wamGetRobotPoseFromToolPose::Response &res)
{ 
  //ROS_INFO("WamTcpIkAlgNode::get_robotPoseCallback:: New Request Received!"); 
  
  //use appropiate mutex to shared variables if necessary 
  //this->alg_.lock(); 
  //this->get_ik_mutex_.enter(); 
  
  // PREDEFINED_TCP TO WAM_TCP
  try{
    ros::Time now = ros::Time::now();
    ros::Duration interval = ros::Duration(1.0);
    if(!listener_.waitForTransform(tool_tcp_str_, robot_tcp_str_, now, interval)){
        ROS_ERROR("Timeout while waiting for transform between frames %s and %s", tool_tcp_str_.c_str(), robot_tcp_str_.c_str()); 
    }
    listener_.lookupTransform(tool_tcp_str_, robot_tcp_str_, now, tcp_H_wam7_);
  }catch (tf::TransformException ex){
    ROS_ERROR("lookup transform error: %s", ex.what());
    return false;
  }

  ROS_INFO("[WamTcpIkAlgNode] %s_H_7 Pose (x, y, z, qx, qy, qz, qw): [ %f, %f, %f, %f, %f, %f, %f ]",tool_tcp_str_.c_str(),
            tcp_H_wam7_.getOrigin().x(),
            tcp_H_wam7_.getOrigin().y(),
            tcp_H_wam7_.getOrigin().z(),
            tcp_H_wam7_.getRotation().x(), 
            tcp_H_wam7_.getRotation().y(), 
            tcp_H_wam7_.getRotation().z(), 
            tcp_H_wam7_.getRotation().w());

  // RECEIVED POSE 
  ROS_INFO("[WamTcpIkAlgNode] Received Pose from frame %s (x, y, z, qx, qy, qz, qw): [ %f, %f, %f, %f, %f, %f, %f ]",
            req.tool_pose.header.frame_id.c_str(),
            req.tool_pose.pose.position.x,
            req.tool_pose.pose.position.y, 
            req.tool_pose.pose.position.z,
            req.tool_pose.pose.orientation.x,
            req.tool_pose.pose.orientation.y,
            req.tool_pose.pose.orientation.z,
            req.tool_pose.pose.orientation.w);

  // frame_id pose
  tf::Quaternion world_quat_tcp( req.tool_pose.pose.orientation.x, req.tool_pose.pose.orientation.y, req.tool_pose.pose.orientation.z, req.tool_pose.pose.orientation.w);
  tf::Vector3 world_pos_tcp( req.tool_pose.pose.position.x, req.tool_pose.pose.position.y, req.tool_pose.pose.position.z);
  tf::Transform received_pose( world_quat_tcp, world_pos_tcp);

  // TF from world wam7
  try{
    ros::Time now = ros::Time::now();
    ros::Duration interval = ros::Duration(1.0);
    if(!listener_.waitForTransform(robot_base_str_, req.tool_pose.header.frame_id, now, interval)){
        ROS_ERROR("Timeout while waiting for transform between frames %s and %s ", robot_base_str_.c_str(), req.tool_pose.header.frame_id.c_str()); 
    }
    listener_.lookupTransform(robot_base_str_, req.tool_pose.header.frame_id, now, world_H_wam7_);
  }catch (tf::TransformException ex){
    ROS_ERROR("lookup transform error: %s", ex.what());
    return false;
  }
 
  world_H_wam7_ *= received_pose;
  world_H_wam7_ *= tcp_H_wam7_;
 
  ROS_INFO("[WamTcpIkAlgNode] 0_H_7 Pose (x, y, z, qx, qy, qz, qw): [ %f, %f, %f, %f, %f, %f, %f ]",
           world_H_wam7_.getOrigin().x(),
           world_H_wam7_.getOrigin().y(),
           world_H_wam7_.getOrigin().z(),
           world_H_wam7_.getRotation().x(), 
           world_H_wam7_.getRotation().y(), 
           world_H_wam7_.getRotation().z(), 
           world_H_wam7_.getRotation().w());

  res.robot_pose.header.frame_id     = robot_base_str_; 
  res.robot_pose.pose.position.x     = world_H_wam7_.getOrigin().x(); 
  res.robot_pose.pose.position.y     = world_H_wam7_.getOrigin().y(); 
  res.robot_pose.pose.position.z     = world_H_wam7_.getOrigin().z(); 
  res.robot_pose.pose.orientation.x  = world_H_wam7_.getRotation().x(); 
  res.robot_pose.pose.orientation.y  = world_H_wam7_.getRotation().y(); 
  res.robot_pose.pose.orientation.z  = world_H_wam7_.getRotation().z(); 
  res.robot_pose.pose.orientation.w  = world_H_wam7_.getRotation().w(); 

  //if(this->alg_.isRunning()) 
  //{ 
    //ROS_INFO("WamTcpIkAlgNode::get_ikCallback: Processin New Request!"); 
    //do operations with req and output on res 
    //res.data2 = req.data1 + my_var; 
  //} 
  //else 
  //{ 
    //ROS_INFO("WamTcpIkAlgNode::get_ikCallback: ERROR: alg is not on run mode yet."); 
  //} 

  //unlock previously blocked shared variables 
  //this->alg_.unlock(); 
  //this->get_ik_mutex_.exit(); 

  return true; 
}


/*  [action callbacks] */

/*  [action requests] */

void WamTcpIkAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  ROS_INFO("Tool tcp frame:  %s", config.tool_tcp.c_str());
  ROS_INFO("Robot tcp frame: %s", config.tool_tcp.c_str());
  tool_tcp_str_ = config.tool_tcp;
  robot_tcp_str_ = config.robot_tcp;
  if(!listener_.frameExists(config.tool_tcp))
      ROS_WARN("Frame %s does not exist, IK won't work until it is published",config.tool_tcp.c_str()); 
  if(!listener_.frameExists(config.robot_tcp))
      ROS_WARN("Frame %s does not exist, IK won't work until it is published",config.robot_tcp.c_str()); 
  this->alg_.unlock();
}

void WamTcpIkAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<WamTcpIkAlgNode>(argc, argv, "wam_tcp_ik_alg_node");
}
