#ifndef _WAM_CALIBRATION_H_
#define _WAM_CALIBRATION_H_

#include <math.h>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include "ros/this_node.h"
#include "ros/names.h"
#include "ros/node_handle.h"
#include "ros/rate.h"


// [publisher subscriber headers]
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "tf/tfMessage.h"
#include "geometry_msgs/PoseStamped.h"

// [service client headers]
#include "iri_wam_common_msgs/pose_move.h"
#include "iri_wam_common_msgs/joints_move.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"

class WamTutorial {
    public:
      WamTutorial();
      
      void mainLoop();

    private:
      bool gocenter;
      int numtargets;
      int targetid;
      std::vector<double> angles;
      ros::NodeHandle nh_;
      ros::Publisher tf_publisher;
      tf::TransformListener listener;
      tf::tfMessage tfMessage_msg;
  //    std::vector<Eigen::Matrix4f> transforms;
      std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > transforms;
  
      ros::Subscriber joint_states_subscriber;
      void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
      ros::Subscriber pose_subscriber;
      void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

      ros::ServiceClient pose_move2_client;
      iri_wam_common_msgs::pose_move pose_move2_srv;
      ros::ServiceClient pose_move_client;
      iri_wam_common_msgs::pose_move pose_move_srv;
      ros::ServiceClient joint_move_client;
      iri_wam_common_msgs::joints_move joint_move_srv;

      void calculate_error(std::string target);
  
  
};
#endif
