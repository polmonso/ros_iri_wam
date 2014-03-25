#ifndef _WAM_IK_KDL_H_
#define _WAM_IK_KDL_H_

#include <math.h>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <string>
#include "ros/this_node.h"
#include "ros/names.h"
#include "ros/node_handle.h"
#include "ros/rate.h"

// kdl includes
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

// [publisher subscriber headers]
#include "sensor_msgs/JointState.h"

// [service client headers]
#include "iri_wam_common_msgs/pose_move.h"
#include "iri_wam_common_msgs/joints_move.h"
#include "iri_wam_common_msgs/wamInverseKinematics.h"

// [action server client headers]

class WamIkKdl {
  private:
  // 
  KDL::Chain wam63_;
  unsigned int num_joints_;
  std::vector<double> currentjoints;

  ros::NodeHandle nh_;
  // [publisher attributes]
  
  // [subscriber attributes]
  ros::Subscriber joint_states_subscriber;
  
  // [server attributes]
  ros::ServiceServer move_in_xyzquat_server;
  ros::ServiceServer print_ik_xyzquat_server;
  
  // [client attributes]
  ros::ServiceClient move_in_joints_client;
  iri_wam_common_msgs::joints_move move_in_joints_srv;

  // [methods]
  void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
  bool move_in_xyzquat_Callback(iri_wam_common_msgs::pose_move::Request &req, iri_wam_common_msgs::pose_move::Response &res);
  bool print_ik_xyzquat_Callback(iri_wam_common_msgs::wamInverseKinematics::Request &req, iri_wam_common_msgs::wamInverseKinematics::Response &res);
  bool ik(std::vector<double> pose, std::vector<double> currentjoints, std::vector<double>& joints);
  
  protected:

  public:
    WamIkKdl();
  
};
#endif
