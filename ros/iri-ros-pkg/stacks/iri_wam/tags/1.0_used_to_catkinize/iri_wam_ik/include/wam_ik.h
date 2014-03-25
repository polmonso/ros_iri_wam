#ifndef _WAM_IK_H_
#define _WAM_IK_H_

#include <math.h>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <string>
#include "ros/this_node.h"
#include "ros/names.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
extern "C" {
  #include "wamik.h" //kaijen wam IK library
}

// [publisher subscriber headers]
#include "sensor_msgs/JointState.h"

// [service client headers]
#include "iri_wam_common_msgs/pose_move.h"
#include "iri_wam_common_msgs/joints_move.h"
#include "iri_wam_common_msgs/wamInverseKinematics.h"

// [action server client headers]

class WamIK {
  private:
    ros::NodeHandle nh_;
    // [publisher attributes]
        sensor_msgs::JointState ik_joints_msg;
    ros::Publisher ik_joints_publisher_;
    Eigen::VectorXd joints_;

    // [subscriber attributes]
    ros::Subscriber joint_states_subscriber;
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
    std::vector<double> currentjoints;

    // [service attributes]
    ros::ServiceServer pose_move_server;
    bool pose_moveCallback(iri_wam_common_msgs::pose_move::Request &req, iri_wam_common_msgs::pose_move::Response &res);
    ros::ServiceServer wamik_server;
    bool wamikCallback(iri_wam_common_msgs::wamInverseKinematics::Request &req, iri_wam_common_msgs::wamInverseKinematics::Response &res);

    // [client attributes]
    ros::ServiceClient joint_move_client;
    iri_wam_common_msgs::joints_move joint_move_srv;
    static bool ik(std::vector<double> pose, std::vector<double> currentjoints, std::vector<double>& joints);

    // [action server attributes]

    // [action client attributes]

  protected:
  public:
    WamIK();
    void ikPub(void);
  
};
#endif
