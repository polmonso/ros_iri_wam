#ifndef _WAM_IKJ_H_
#define _WAM_IKJ_H_

#include <Eigen/Geometry>
#include <math.h>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <string>
#include "ros/this_node.h"
#include "ros/names.h"
#include "ros/node_handle.h"
#include "ros/rate.h"

// [publisher subscriber headers]
#include "sensor_msgs/JointState.h"

// [service client headers]
#include "iri_wam_common_msgs/pose_move.h"
#include "iri_wam_common_msgs/joints_move.h"
#include "iri_wam_common_msgs/wamInverseKinematics.h"

// [action server client headers]

class WamIKJ {
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
	// capçaleres

	static void fkine(Eigen::VectorXd theta,Eigen::MatrixXd& Tf,Eigen::MatrixXd& Jn);
	static void errortrans(Eigen::MatrixXd Tcurrent, Eigen::MatrixXd Tdesired, Eigen::VectorXd& errc);
	static void filtersols(Eigen::MatrixXd qsol,Eigen::VectorXd& sol);
	static void DHmatrix(double alpha, double a, double d,double& theta, Eigen::MatrixXd& T);
	static void skewop(Eigen::Matrix3d& M,Eigen::Vector3d v);
	static void potentialfunction(Eigen::VectorXd qref,Eigen::VectorXd& q, double& potq);
	static void sphericalikine(Eigen::MatrixXd T0,Eigen::MatrixXd T4, Eigen::Vector2d& q5,Eigen::Vector2d& q6,Eigen::Vector2d& q7);
	static void soltrig(double a, double b, double c, Eigen::MatrixXd& q2sol2);
	static void dotprod(Eigen::MatrixXd u,Eigen::MatrixXd v,double& p);
	static Eigen::MatrixXd crossprod(Eigen::MatrixXd u,Eigen::MatrixXd v);
	static void svdJ(Eigen::MatrixXd J,Eigen::MatrixXd& U,Eigen::MatrixXd& V,Eigen::MatrixXd& S);
	static void pseudoinverse(Eigen::MatrixXd J,Eigen::MatrixXd& Ji,Eigen::MatrixXd& U,Eigen::MatrixXd& V,Eigen::MatrixXd& S);
	static void Dpseudoinverse(Eigen::MatrixXd J,Eigen::MatrixXd& Ji,double lambda);
	static void Fpseudoinverse(Eigen::MatrixXd J,Eigen::MatrixXd& Ji,double lambda, double eps);
	static void CFpseudoinverse(Eigen::MatrixXd J,Eigen::MatrixXd& Ji,double nu,double s0);

	static Eigen::VectorXd pi2piwam(Eigen::VectorXd q);
	static Eigen::MatrixXd rcpinv(Eigen::MatrixXd Q,Eigen::MatrixXd W);
	static Eigen::MatrixXd lcpinv(Eigen::MatrixXd Q,Eigen::MatrixXd W);
	static Eigen::MatrixXd cpinv(Eigen::MatrixXd J,Eigen::MatrixXd H);
	static void HjlCONT(Eigen::VectorXd q,Eigen::MatrixXd& H,double b);
	static double fubeta(double x,double beta);
	static void clampjoints(Eigen::VectorXd q,Eigen::MatrixXd& H);
	static  void dhfunct(Eigen::VectorXd q,Eigen::VectorXd& dH);
	static  void manipgrad(Eigen::VectorXd q,double& man,Eigen::VectorXd& gradman);
	static  void manipulability(Eigen::MatrixXd J,double& man);
    // [action server attributes]

    // [action client attributes]

  protected:
  public:
    WamIKJ();
    void ikPub(void);
  
};
#endif
