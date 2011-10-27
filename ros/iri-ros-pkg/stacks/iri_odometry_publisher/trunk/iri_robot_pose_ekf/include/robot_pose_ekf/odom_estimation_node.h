/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#ifndef __ODOM_ESTIMATION_NODE__
#define __ODOM_ESTIMATION_NODE__

// ros stuff
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "odom_estimation.h"
#include <robot_pose_ekf/GetStatus.h>

// messages
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <boost/thread/mutex.hpp>

// log files
#include <fstream>

namespace estimation
{

/** \mainpage
 *  \htmlinclude manifest.html
 *
 * <b>Package Summary</b>
 * This package provides two main classes:
 *  1) OdomEstimation performs all sensor fusion operations, and
 *  2) OdomEstimationNode provides a ROS wrapper around OdomEstimation
*/

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
typedef boost::shared_ptr<nav_msgs::Odometry const> CmpConstPtr;
typedef boost::shared_ptr<nav_msgs::Odometry const> IcpConstPtr;
typedef boost::shared_ptr<nav_msgs::Odometry const> GpsConstPtr;
typedef boost::shared_ptr<geometry_msgs::Twist const> VelConstPtr;

class OdomEstimationNode
{
public:
  /// constructor
  OdomEstimationNode();

  /// destructor
  virtual ~OdomEstimationNode();

private:
  /// the mail filter loop that will be called periodically
  void spin(const ros::TimerEvent& e);

  /// callback function for odo data
  void odomCallback(const OdomConstPtr& odom);

  /// callback function for icp data
  void icpCallback(const IcpConstPtr& imu);

  /// callback function for compass data
  void cmpCallback(const CmpConstPtr& vo);

  /// callback function for gps data
  void gpsCallback(const GpsConstPtr& imu);

  void templateCallback(const std::string& name, const OdomConstPtr& odom,
                        uint & counter, const bool& used, ros::Time& stamp,
                        ros::Time& time, tf::Transform & meas,
                        MatrixWrapper::SymmetricMatrix& covariance,
                        const std::string & child_id, bool& active,
                        bool& initializing, ros::Time& init_stamp,
                        std::ofstream & file);

  /// get the status of the filter
  bool getStatus(robot_pose_ekf::GetStatus::Request& req, robot_pose_ekf::GetStatus::Response& resp);

  ros::NodeHandle node_;
  ros::Timer timer_;
  ros::Publisher pose_pub_;
  ros::Subscriber odom_sub_, icp_sub_, cmp_sub_, gps_sub_;
  ros::ServiceServer state_srv_;

  // ekf filter
  OdomEstimation my_filter_;

  // estimated robot pose message to send
  geometry_msgs::PoseWithCovarianceStamped  output_;
  std::string output_frame_;

  // robot state
  tf::TransformListener    robot_state_;
  tf::TransformBroadcaster odom_broadcaster_;

  // vectors
  tf::Transform odom_meas_, icp_meas_, cmp_meas_, gps_meas_;

  // time
  ros::Time odom_time_,       icp_time_,       cmp_time_,       gps_time_;
  ros::Time odom_stamp_,      icp_stamp_,      cmp_stamp_,      gps_stamp_, filter_stamp_;
  ros::Time odom_init_stamp_, icp_init_stamp_, cmp_init_stamp_, gps_init_stamp_;

  // state
  bool odom_active_,       icp_active_,       cmp_active_,       gps_active_;
  bool odom_used_,         icp_used_,         cmp_used_,         gps_used_;
  bool odom_initializing_, icp_initializing_, cmp_initializing_, gps_initializing_;;

  double timeout_; //!< sensor timeout

  MatrixWrapper::SymmetricMatrix odom_covariance_, icp_covariance_, cmp_covariance_, gps_covariance_;

  // debugging
  bool debug_, self_diagnose_;
  std::ofstream odom_file_, icp_file_, cmp_file_, gps_file_, corr_file_, time_file_, extra_file_;

  // counters
  unsigned int odom_callback_counter_, icp_callback_counter_, cmp_callback_counter_, gps_callback_counter_, ekf_sent_counter_;

}; // class

}; // namespace

#endif
