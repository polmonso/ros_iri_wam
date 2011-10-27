// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author Joan Perez
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// IMPORTANT NOTE: This code has been generated through a script from the
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _generic_odometry_h_
#define _generic_odometry_h_

#include "iri_base_algorithm/iri_base_algorithm.h"
#include <tf/transform_broadcaster.h>

// [include algorithm common interface]
#include <odometry_core/odometry_iri_base.h>

// [publisher subscriber headers]
#include <nav_msgs/Odometry.h>

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Driver Class
 *
 * This class inherits from the IRI Core class IriBaseNodeDriver<IriBaseDriver>,
 * to provide an execution thread to the driver object. A complete framework
 * with utilites to test the node functionallity or to add diagnostics to
 * specific situations is also given. The inherit template design form allows
 * complete access to any IriBaseDriver object implementation.
 *
 * As mentioned, tests in the different driver states can be performed through
 * class methods such as addNodeOpenedTests() or addNodeRunningTests(). Tests
 * common to all nodes may be also executed in the pattern class IriBaseNodeDriver.
 * Similarly to the tests, diagnostics can easyly be added. See ROS Wiki for
 * more details:
 * http://www.ros.org/wiki/diagnostics/ (Tutorials: Creating a Diagnostic Analyzer)
 * http://www.ros.org/wiki/self_test/ (Example: Self Test)
 */
template <class Algorithm, class MsgType>
class GenericOdometry : public algorithm_base::IriBaseAlgorithm<Algorithm>
{
  //TODO: Config is already defined in IriBaseAlgorithm
  // should not be necessary to redefined in here
  public:
    typedef typename Algorithm::Config Config;

  private:
    // [publisher attributes]
    ros::Publisher odom_publisher_;

    // [subscriber attributes]
    ros::Subscriber platform_status_subscriber_;
    void platform_status_callback(const MsgType& msg);

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

    bool is_first_msg_;
    std::string tf_prefix_;
    std::string odom_id_;
    std::string base_link_id_;
    bool publish_tf_;

  public:
   /**
    * \brief Constructor
    *
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    GenericOdometry();

   /**
    * \brief Destructor
    *
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~GenericOdometry();

  protected:
   /**
    * \brief odometry tf broadcaster
    *
    * This attribute is used to broadcast odometry based robot position through
    * tf transforms.
    */
    tf::TransformBroadcaster odom_broadcaster_;

   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]

    // [test functions]
};

template <class Algorithm, class MsgType>
GenericOdometry<Algorithm, MsgType>::GenericOdometry() :
  is_first_msg_(true)
{
  ROS_DEBUG("GenericOdometry Constructor");

  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  std::string tf_prefix;
  this->public_node_handle_.template param<std::string>("tf_prefix",    tf_prefix_,     "");
  this->public_node_handle_.template param<std::string>("odom_id",      odom_id_,      "odom" );
  this->public_node_handle_.template param<std::string>("base_link_id", base_link_id_, "base_link" );
  this->public_node_handle_.template param<bool>       ("publish_tf",   publish_tf_,   true  );

  // push down frame ids if necessary
  ROS_INFO("New 3");
  if( !tf_prefix_.empty() )
  {
    odom_id_      = tf_prefix_ + "/" + odom_id_;
    base_link_id_ = tf_prefix_ + "/" + base_link_id_;
  }
  ROS_INFO("tf_prefix_=%s base_link_id_=%s",tf_prefix_.c_str(),base_link_id_.c_str());

  // [init publishers]
  this->odom_publisher_ = this->public_node_handle_.template advertise<nav_msgs::Odometry>("odom", 100);

  // [init subscribers]
  this->platform_status_subscriber_ = this->public_node_handle_.subscribe("platform_status", 100,
                                               &GenericOdometry<Algorithm, MsgType>::platform_status_callback, this);
  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

template <class Algorithm, class MsgType>
GenericOdometry<Algorithm, MsgType>::~GenericOdometry()
{
  // [free dynamic memory]
}

template <class Algorithm, class MsgType>
void GenericOdometry<Algorithm, MsgType>::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
template <class Algorithm, class MsgType>
void GenericOdometry<Algorithm, MsgType>::platform_status_callback(const MsgType& msg)
{
  //ROS_DEBUG("New Platform status message received. Computing Odometry");

  if(is_first_msg_)
  {
    is_first_msg_ = false;
    this->alg_.setLastTime(ros::Time::now());
  }

  this->alg_.computeOdometry(msg);

  //send the transform
  if(publish_tf_)
  {
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans_msg;
    odom_trans_msg.header.stamp    = this->alg_.getOdomTime();
    odom_trans_msg.header.frame_id = odom_id_;
    odom_trans_msg.child_frame_id  = base_link_id_;
    odom_trans_msg.transform       = this->alg_.getTransform();

    this->odom_broadcaster_.sendTransform(odom_trans_msg);
    ROS_DEBUG("Sending Odometry Transform");
  }

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp     = this->alg_.getOdomTime();
  odom_msg.header.frame_id  = odom_id_;
  odom_msg.child_frame_id   = base_link_id_;
  odom_msg.pose             = this->alg_.getPose();
  odom_msg.twist            = this->alg_.getTwist();

  //publish the message
  this->odom_publisher_.publish(odom_msg);
  //ROS_DEBUG("Sending Odometry");
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

template <class Algorithm, class MsgType>
void GenericOdometry<Algorithm, MsgType>::node_config_update(Config &config, uint32_t level)
{
}

template <class Algorithm, class MsgType>
void GenericOdometry<Algorithm, MsgType>::addNodeDiagnostics(void)
{
}

#endif
