#include "wam_tcp_ik_alg_node.h"

WamTcpIkAlgNode::WamTcpIkAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<WamTcpIkAlgorithm>()
{
  //string for port names
    frame_tcp_str_ = "/bhand_tcp";
  
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  get_ik_server_ = this->public_node_handle_.advertiseService( "get_wam_ik", &WamTcpIkAlgNode::get_ikCallback, this);
  
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
  //ROS_INFO("WamTcpIkAlgNode::get_ikCallback: New Request Received!"); 
  
  //use appropiate mutex to shared variables if necessary 
  //this->alg_.lock(); 
  //this->get_ik_mutex_.enter(); 
  
  ROS_INFO("[WamTcpIkAlgNode] Received Pose (x, y, z, qx, qy, qz, qw): [ %f, %f, %f, %f, %f, %f, %f ]",
            req.pose.position.x,
            req.pose.position.y, 
            req.pose.position.z,
            req.pose.orientation.x,
            req.pose.orientation.y,
            req.pose.orientation.z,
            req.pose.orientation.w);
  
  try{
    ros::Time now = ros::Time::now();
    ros::Duration interval = ros::Duration(1.0);
    if(!listener_.waitForTransform(frame_tcp_str_, "/wam_tcp", now, interval)){
        ROS_ERROR("Timeout while waiting for transform between frames %s and /wam_tcp/ ", frame_tcp_str_.c_str()); 
    }
    listener_.lookupTransform(frame_tcp_str_, "/wam_tcp", now, tcp_H_wam7_);
  }catch (tf::TransformException ex){
    ROS_ERROR("lookup transform error: %s", ex.what());
    return false;
  }

  ROS_INFO("[WamTcpIkAlgNode] %s_H_7 Pose (x, y, z, qx, qy, qz, qw): [ %f, %f, %f, %f, %f, %f, %f ]",frame_tcp_str_.c_str(),
            tcp_H_wam7_.getOrigin().x(),
            tcp_H_wam7_.getOrigin().y(),
            tcp_H_wam7_.getOrigin().z(),
            tcp_H_wam7_.getRotation().x(), 
            tcp_H_wam7_.getRotation().y(), 
            tcp_H_wam7_.getRotation().z(), 
            tcp_H_wam7_.getRotation().w());
 
  tf::Quaternion world_quat_tcp( req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w);
  tf::Vector3 world_pos_tcp( req.pose.position.x, req.pose.position.y, req.pose.position.z);
  tf::Transform world_H_wam7( world_quat_tcp, world_pos_tcp);
  
  world_H_wam7 *= tcp_H_wam7_;
  
  base_pose_msg_.request.pose.position.x    = world_H_wam7.getOrigin().x(); 
  base_pose_msg_.request.pose.position.y    = world_H_wam7.getOrigin().y(); 
  base_pose_msg_.request.pose.position.z    = world_H_wam7.getOrigin().z(); 
  base_pose_msg_.request.pose.orientation.x = world_H_wam7.getRotation().x(); 
  base_pose_msg_.request.pose.orientation.y = world_H_wam7.getRotation().y(); 
  base_pose_msg_.request.pose.orientation.z = world_H_wam7.getRotation().z(); 
  base_pose_msg_.request.pose.orientation.w = world_H_wam7.getRotation().w(); 

  ROS_INFO("[WamTcpIkAlgNode] 0_H_7 Pose (x, y, z, qx, qy, qz, qw): [ %f, %f, %f, %f, %f, %f, %f ]",
           world_H_wam7.getOrigin().x(),
           world_H_wam7.getOrigin().y(),
           world_H_wam7.getOrigin().z(),
           world_H_wam7.getRotation().x(), 
           world_H_wam7.getRotation().y(), 
           world_H_wam7.getRotation().z(), 
           world_H_wam7.getRotation().w());

  bool result;
  if(get_ik_client_.call(base_pose_msg_)){
    res.joints.position.resize(7);
    for(int ii=0; ii < 7; ++ii)
      res.joints.position[ii] = base_pose_msg_.response.joints.position[ii];
    ROS_INFO("[WamTcpIkAlgNode] Joints readings: (%f, %f, %f, %f, %f, %f, %f)",
        base_pose_msg_.response.joints.position[0], 
        base_pose_msg_.response.joints.position[1], 
        base_pose_msg_.response.joints.position[2], 
        base_pose_msg_.response.joints.position[3], 
        base_pose_msg_.response.joints.position[4],
        base_pose_msg_.response.joints.position[5], 
        base_pose_msg_.response.joints.position[6] );
    result = true;
  }else{
    ROS_ERROR("Failed to call service %s", get_ik_client_.getService().c_str());
    result = false;
  }
  

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

  return result; 
}

/*  [action callbacks] */

/*  [action requests] */

void WamTcpIkAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  ROS_INFO("tcp frame change %s",config.frame_tcp.c_str());
  frame_tcp_str_ = config.frame_tcp;
  if(!listener_.frameExists(config.frame_tcp))
      ROS_WARN("Frame %s does not exist, IK won't work until it is published",config.frame_tcp.c_str()); 
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
