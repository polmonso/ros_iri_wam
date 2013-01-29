#ifndef _KUKA_IKAC_H_
#define _KUKA_IKAC_H_

#include <math.h>
#include <cstddef>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <string>
#include "ros/this_node.h"
#include "ros/names.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include <tf/transform_listener.h>

// [publisher subscriber headers]
#include "sensor_msgs/JointState.h"

// [service client headers]
#include "iri_wam_common_msgs/pose_move.h"
#include "iri_wam_common_msgs/joints_move.h"
#include "iri_wam_common_msgs/wamInverseKinematics.h"
#include "iri_wam_common_msgs/wamInverseKinematicsFromPose.h"

// [action server client headers]

class KukaIKAC {
  private:
    double step_, d1, d3, d5, d7;
    Eigen::MatrixXd DHparam_;
    Eigen::MatrixXd Qlim_;
    int IMAX_,checksol_;
    std::vector<double> currentjoints;

    ros::NodeHandle nh_;
    
    tf::TransformListener listener_; 
    tf::StampedTransform tcp_H_wam7_;

    // [publisher attributes]
    sensor_msgs::JointState ik_joints_msg;
    ros::Publisher ik_joints_publisher_;
    Eigen::VectorXd joints_;

    // [subscriber attributes]
    ros::Subscriber joint_states_subscriber;
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
    std::vector<double> currentjoints_;

    // [service attributes]
    ros::ServiceServer kukaik_server_fromPose;
    bool kukaikCallbackFromPose(iri_wam_common_msgs::wamInverseKinematicsFromPose::Request &req, iri_wam_common_msgs::wamInverseKinematicsFromPose::Response &res);
    ros::ServiceServer kukaik_server;
    bool kukaikCallback(iri_wam_common_msgs::wamInverseKinematics::Request &req, iri_wam_common_msgs::wamInverseKinematics::Response &res);

    // [client attributes]
    ros::ServiceClient joint_move_client;
    iri_wam_common_msgs::joints_move joint_move_srv;
    // capçaleres
    bool ik(std::vector<double> pose, std::vector<double> currentjoints, std::vector<double>& joints);
	  void filtersols(Eigen::MatrixXd qsol,Eigen::VectorXd& sol);
	  void DHmatrix(double alpha,double a, double d,double& theta, Eigen::MatrixXd& T);
	  void basicsols(Eigen::MatrixXd qsol,Eigen::MatrixXd& qaux,Eigen::VectorXd sol);
	  void optimizesol(Eigen::MatrixXd q,Eigen::MatrixXd T0, Eigen::VectorXd& qbest,double& potq,Eigen::VectorXd qref);
	  void skewop(Eigen::Matrix3d& M,Eigen::Vector3d v);
	  void getq1(Eigen::VectorXd cjoints, float i, double& q10);
	  double pi2pi(double x);
	  void exactikine(Eigen::MatrixXd T0, double q10,Eigen::VectorXd cjoints,  Eigen::MatrixXd& qsol,Eigen::VectorXd&sol);
	  void bestsol(Eigen::MatrixXd qsol,Eigen::VectorXd sol,Eigen::VectorXd qref,Eigen::VectorXd& q,double& potq);
	  void potentialfunction(Eigen::VectorXd qref,Eigen::VectorXd& q, double& potq);
	  void sphericalikine(Eigen::MatrixXd T0,Eigen::MatrixXd T4, Eigen::VectorXd cjoints, Eigen::Vector2d& q5,Eigen::Vector2d& q6,Eigen::Vector2d& q7);
	  void initialelbows(Eigen::VectorXd q,Eigen::MatrixXd T0,Eigen::VectorXd& PJ);
	  void soltrig(double a, double b, double c, Eigen::MatrixXd& q2sol2);
    // [action server attributes]

    // [action client attributes]

  protected:
  public:
    KukaIKAC();
    void ikPub(void);
  
};
#endif
