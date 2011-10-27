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
*   * Redistributions of source code must retain the above copyright
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

#include <robot_pose_ekf/odom_estimation_node.h>


using namespace MatrixWrapper;
using namespace std;
using namespace ros;
using namespace tf;


//static const double EPS = 1e-5; // TODO aixo serveix per alguna cosa?


//#define __EKF_DEBUG_FILE__

namespace estimation
{

  //  CONSTRUCTOR
  //
  OdomEstimationNode::OdomEstimationNode()
    : odom_active_(false),
      icp_active_(false),
      cmp_active_(false),
      gps_active_(false),
      odom_initializing_(false),
      icp_initializing_(false),
      cmp_initializing_(false),
      gps_initializing_(false),
      odom_covariance_(6),
      icp_covariance_(3),
      cmp_covariance_(3),
      gps_covariance_(3),
      odom_callback_counter_(0),
      icp_callback_counter_(0),
      cmp_callback_counter_(0),
      gps_callback_counter_(0),
      ekf_sent_counter_(0)
  {
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    // paramters
    nh_private.param("output_frame",   output_frame_,  std::string("odom_combined"));
    nh_private.param("sensor_timeout", timeout_,       1.0);
    nh_private.param("odom_used",      odom_used_,     true);
    nh_private.param("icp_used",       icp_used_,      true);
    nh_private.param("cmp_used",       cmp_used_,      true);
    nh_private.param("gps_used",       gps_used_,      true);
    nh_private.param("debug",          debug_,         false);
    nh_private.param("self_diagnose",  self_diagnose_, false);
    double freq;
    nh_private.param("freq", freq, 30.0);

    timer_ = nh_private.createTimer(ros::Duration(1.0/max(freq,1.0)), &OdomEstimationNode::spin, this);

    // advertise our estimation
    pose_pub_ = nh_private.advertise<geometry_msgs::PoseWithCovarianceStamped>(output_frame_, 10);

    // initialize
    filter_stamp_ = Time::now();

    // subscribe to odom messages
    if (odom_used_){
      ROS_INFO("Odom sensor can be used");
      odom_sub_ = nh.subscribe("odom_data", 10, &OdomEstimationNode::odomCallback, this);
    }
    else ROS_INFO("Odom sensor will NOT be used");

    // subscribe to icp messages
    if (icp_used_){
      ROS_INFO("ICP Odom can be used");
      icp_sub_ = nh.subscribe("icp_data", 10,  &OdomEstimationNode::icpCallback, this);
    }
    else ROS_INFO("ICP Odom will NOT be used");

    // subscribe to compass messages
    if (cmp_used_){
      ROS_INFO("Compass sensor can be used");
      cmp_sub_ = nh.subscribe("cmp_data", 10, &OdomEstimationNode::cmpCallback, this);
    }
    else ROS_INFO("Compass sensor will NOT be used");

    // subscribe to gps messages
    if (gps_used_){
      ROS_INFO("GPS sensor can be used");
      gps_sub_ = nh.subscribe("gps_data", 10, &OdomEstimationNode::gpsCallback, this);
    }
    else ROS_INFO("GPS sensor will NOT be used");

    // publish state service
    state_srv_ = nh_private.advertiseService("get_status", &OdomEstimationNode::getStatus, this);

    if (debug_){
      // open files for debugging
      odom_file_.open( "/tmp/odom_file.txt");
      icp_file_.open(  "/tmp/icp_file.txt");
      cmp_file_.open(  "/tmp/cmp_file.txt");
      gps_file_.open(  "/tmp/gps_file.txt");
      corr_file_.open( "/tmp/corr_file.txt");
      time_file_.open( "/tmp/time_file.txt");
    }
  };

  //  DESTRUCTOR
  //
  OdomEstimationNode::~OdomEstimationNode()
  {
    if (debug_)
    {
      // close files for debugging
      odom_file_.close();
      icp_file_.close();
      cmp_file_.close();
      gps_file_.close();
      corr_file_.close();
      time_file_.close();
    }
  };

  //  ODOM
  //
  void OdomEstimationNode::odomCallback(const OdomConstPtr& odom)
  {
    templateCallback("Odometry", odom, odom_callback_counter_, odom_used_, odom_stamp_,
                     odom_time_, odom_meas_, odom_covariance_, "wheelodom", odom_active_,
                     odom_initializing_, odom_init_stamp_, odom_file_);
  };

  //  ICP
  //
  void OdomEstimationNode::icpCallback(const OdomConstPtr& odom)
  {
    templateCallback("ICP", odom, icp_callback_counter_, icp_used_, icp_stamp_,
                     icp_time_, icp_meas_, icp_covariance_, "wheelicp", icp_active_,
                     icp_initializing_, icp_init_stamp_, icp_file_);
  };
  
  //  COMPASS
  //
  void OdomEstimationNode::cmpCallback(const OdomConstPtr& odom)
  {
    templateCallback("Compass", odom, cmp_callback_counter_, cmp_used_, cmp_stamp_,
                     cmp_time_, cmp_meas_, cmp_covariance_, "wheelcmp", cmp_active_,
                     cmp_initializing_, cmp_init_stamp_, cmp_file_);
  };
  
  //  GPS
  //
  void OdomEstimationNode::gpsCallback(const OdomConstPtr& odom)
  {
    templateCallback("GPS", odom, gps_callback_counter_, gps_used_, gps_stamp_,
                     gps_time_, gps_meas_, gps_covariance_, "wheelgps", gps_active_,
                     gps_initializing_, gps_init_stamp_, gps_file_);
  };
  
  //  TEMPLATE
  //
  void OdomEstimationNode::templateCallback(const std::string& name,
                      const OdomConstPtr& odom, uint & counter,
                      const bool & used, ros::Time& stamp,
                      ros::Time& time, tf::Transform & meas,
                      MatrixWrapper::SymmetricMatrix& covariance,
                      const std::string & child_id, bool& active,
                      bool& initializing, ros::Time& init_stamp,
                      std::ofstream & file)
  {
    //
    counter++;
    //
    ROS_DEBUG("%s callback at time %f ", name.c_str(), ros::Time::now().toSec());
    assert(used);
    //
    // GET DATA
    // Timestamps (message stamp and arrival time)
    stamp = odom->header.stamp;
    time  = Time::now();
    //
    // POSE
    // transform data to TF system -> MEAS
    Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
    meas = Transform(q, Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z));
    //printf("\n");
    // fill covariance matrix -> COV
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
      {covariance(i+1, j+1) = odom->pose.covariance[6*i+j];
       //printf("%f ",odom->pose.covariance[6*i+j]);
      }
      //printf("\n");
    //
    // ADD to FILTER
    my_filter_.addMeasurement( StampedTransform(meas.inverse(), odom_stamp_, "base_footprint", child_id), odom_covariance_);
    //
    //
    // INITIALIZATION and ACTIVATION
    if (!active)
    {
      if (!initializing)
      {
        initializing = true;
        init_stamp = stamp;
        ROS_INFO("Initializing %s sensor",name.c_str());
      }
      if ( filter_stamp_ >= init_stamp)
      {
        active       = true;
        initializing = false;
        ROS_INFO("%s sensor activated",name.c_str());
      } else {
        ROS_INFO("Waiting to activate %s, because Odom measurements are still %f sec in the future."
                  , name.c_str(), (odom_init_stamp_ - filter_stamp_).toSec() );
      }
    }
  };

  //  FILTER LOOP
  //
  void OdomEstimationNode::spin(const ros::TimerEvent& e)
  {
    ROS_DEBUG("Spin function at time %f", ros::Time::now().toSec());

    // check for timing problems
    if ( (odom_initializing_ || odom_active_) && (cmp_initializing_ || cmp_active_) )
    {
      double diff = fabs( Duration(odom_stamp_ - cmp_stamp_).toSec() );
      if (diff > 1.0) ROS_ERROR("Timestamps of odometry and compass are %f seconds apart.", diff);
    }

    // initial value for filter stamp; keep this stamp when no sensors are active
    filter_stamp_ = Time::now();

    // check which sensors are still active
    if ((odom_active_ || odom_initializing_) &&
        (Time::now() - odom_time_).toSec() > timeout_){
      odom_active_ = false; odom_initializing_ = false;
      ROS_INFO("Odom not active any more");
    }
    if ((icp_active_ || icp_initializing_) &&
        (Time::now() - icp_time_).toSec() > timeout_){
      icp_active_ = false;  icp_initializing_ = false;
      ROS_INFO("ICP not active any more");
    }
    if ((cmp_active_ || cmp_initializing_) &&
      (Time::now() - cmp_time_).toSec() > timeout_){
      cmp_active_ = false;  cmp_initializing_ = false;
      ROS_INFO("Compass not active any more");
    }
    if ((gps_active_ || gps_initializing_) &&
      (Time::now() - gps_time_).toSec() > timeout_){
      gps_active_ = false;  gps_initializing_ = false;
      ROS_INFO("GPS not active any more");
    }


    // only update filter when one of the sensors is active
    if (odom_active_ || icp_active_ || cmp_active_ || gps_active_){

      // update filter at time where all sensor measurements are available
      if (odom_active_) filter_stamp_ = min(filter_stamp_, odom_stamp_);
      if (icp_active_)  filter_stamp_ = min(filter_stamp_, icp_stamp_);
      if (cmp_active_)  filter_stamp_ = min(filter_stamp_, cmp_stamp_);
      if (gps_active_)  filter_stamp_ = min(filter_stamp_, gps_stamp_);

      // update filter
      if ( my_filter_.isInitialized() )  {
        bool diagnostics = true;
        if (my_filter_.update(odom_active_, icp_active_, cmp_active_,
                              gps_active_, filter_stamp_, diagnostics)){

          // output most recent estimate and relative covariance
          my_filter_.getEstimate(output_);
          pose_pub_.publish(output_);
          ekf_sent_counter_++;

          // broadcast most recent estimate to TransformArray
          StampedTransform tmp;
          my_filter_.getEstimate(ros::Time(), tmp);
          if(!odom_active_ || !cmp_active_)
            tmp.getOrigin().setZ(0.0);
          odom_broadcaster_.sendTransform(StampedTransform(tmp, tmp.stamp_, output_frame_, "base_footprint"));

          if (debug_){
            // write to file
            ColumnVector estimate;
            my_filter_.getEstimate(estimate);
            for (unsigned int i=1; i<=6; i++)
              corr_file_ << estimate(i) << " ";
            corr_file_ << endl;
          }
        }
        if (self_diagnose_ && !diagnostics)
          ROS_WARN("Robot pose ekf diagnostics discovered a potential problem");
      }
      // initialize filer with odometry frame
      if ( odom_active_ && !my_filter_.isInitialized()){
        my_filter_.initialize(odom_meas_, odom_stamp_);
        ROS_INFO("Kalman filter initialized with odom measurement");
      }
      else if ( icp_active_ && !my_filter_.isInitialized()){
        my_filter_.initialize(icp_meas_, icp_stamp_);
        ROS_INFO("Kalman filter initialized with ICP measurement");
      }
    }
  };


bool OdomEstimationNode::getStatus(robot_pose_ekf::GetStatus::Request& req, robot_pose_ekf::GetStatus::Response& resp)
{
//   stringstream ss;
//   ss << "Input:" << endl;
//   ss << " * Odometry sensor" << endl;
//   ss << "   - is "; if (!odom_active_) ss << "NOT "; ss << "active" << endl;
//   ss << "   - received " << odom_callback_counter_ << " messages" << endl;
//   ss << "   - listens to topic " << odom_sub_.getTopic() << endl;
//   ss << " * IMU sensor" << endl;
//   ss << "   - is "; if (!imu_active_) ss << "NOT "; ss << "active" << endl;
//   ss << "   - received " << imu_callback_counter_ << " messages" << endl;
//   ss << "   - listens to topic " << imu_sub_.getTopic() << endl;
//   ss << " * Visual Odometry sensor" << endl;
//   ss << "   - is "; if (!vo_active_) ss << "NOT "; ss << "active" << endl;
//   ss << "   - received " << vo_callback_counter_ << " messages" << endl;
//   ss << "   - listens to topic " << vo_sub_.getTopic() << endl;
//   ss << "Output:" << endl;
//   ss << " * Robot pose ekf filter" << endl;
//   ss << "   - is "; if (!my_filter_.isInitialized()) ss << "NOT "; ss << "active" << endl;
//   ss << "   - sent " << ekf_sent_counter_ << " messages" << endl;
//   ss << "   - pulishes on topics " << pose_pub_.getTopic() << " and /tf and" << endl;
//   ss << "   - pulishes on topics " << pose2_pub_.getTopic() <<  endl;
//   resp.status = ss.str();
  return true;
}

}; // namespace

// ----------
// -- MAIN --
// ----------
using namespace estimation;
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "robot_pose_ekf");

  // create filter class
  OdomEstimationNode my_filter_node;

  ros::spin();

  return 0;
}




/*
//  ODOM
//
void OdomEstimationNode::odomCallback(const OdomConstPtr& odom)
{
  //
  odom_callback_counter_++;
  //
  ROS_INFO("Odom callback at time %f ", ros::Time::now().toSec());
  assert(odom_used_);
  //
  // GET DATA
  // Timestamps (message stamp and arrival time)
  odom_stamp_ = odom->header.stamp;
  odom_time_  = Time::now();
  //
  // POSE
  // transform data to TF system -> MEAS
  Quaternion q;
  tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
  odom_meas_  = Transform(q, Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z));
  // fill covariance matrix -> COV
  for (unsigned int i=0; i<6; i++)
    for (unsigned int j=0; j<6; j++)
      odom_covariance_(i+1, j+1) = odom->pose.covariance[6*i+j];
    //
      // ADD to FILTER
      my_filter_.addMeasurement( StampedTransform(odom_meas_.inverse(), odom_stamp_, "base_footprint", "wheelodom"), odom_covariance_);
      //
      //
      // INITIALIZATION and ACTIVATION
      if (!odom_active_)
      {
        if (!odom_initializing_)
        {
          odom_initializing_ = true;
          odom_init_stamp_ = odom_stamp_;
          ROS_INFO("Initializing Odom sensor");
        }
        if ( filter_stamp_ >= odom_init_stamp_)
        {
          odom_active_ = true;
          odom_initializing_ = false;
          ROS_INFO("Odom sensor activated");
        } else {
          ROS_INFO("Waiting to activate Odom, because Odom measurements are still %f sec in the future.", (odom_init_stamp_ - filter_stamp_).toSec());
        }
      }
      //
      // DEBUG STUFF
      if (debug_)
      {
        // write to file
        double tmp, yaw;
        odom_meas_.getBasis().getEulerZYX(yaw, tmp, tmp);
        odom_file_ << odom_meas_.getOrigin().x() << " " << odom_meas_.getOrigin().y() << "  " << yaw << "  " << endl;
      }
};
*/
