#ifndef _WAM_FK_H_
#define _WAM_FK_H_

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

class WamFK {
    public:
      WamFK();
      
      void forwardKinematics(const sensor_msgs::JointState::ConstPtr& msg,geometry_msgs::Pose& pose);

      void dhMatrix(float a, float alpha, float d, float theta, Eigen::Matrix4f& T);
      void tfPub(void);

    private:
      ros::NodeHandle nh_;
      ros::Publisher tf_publisher;
      tf::tfMessage tfMessage_msg;
      void fillTfmessage(int joint, const Eigen::Matrix4f T);
  //    std::vector<Eigen::Matrix4f> transforms;
      std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > transforms;
  
      ros::Subscriber joint_states_subscriber;

      std::string wambase_;
      std::string wam0_;
      std::string wam1_;
      std::string wam2_;
      std::string wam3_;
      std::string wam4_;
      std::string wam5_;
      std::string wam6_;
      std::string wam7_;

      void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
  
      template<typename T>
      void quatToMatrix(Eigen::Matrix<T,4,4>& m,
                        const Eigen::Quaternion<T> qrot);
  
};
#endif
