// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
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

#ifndef _wam_controller_driver_node_h_
#define _wam_controller_driver_node_h_

#include <iri_base_driver/iri_base_driver_node.h>
#include "wam_controller_driver.h"
#include <Eigen/Dense>

// [publisher subscriber headers]
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

// [service client headers]
#include <iri_wam_common_msgs/wamholdon.h>
#include <iri_wam_common_msgs/joints_move.h>

// [action server client headers]
#include <iri_wam_common_msgs/DMPTrackerAction.h>
#include <iri_action_server/iri_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef enum {TRAJ_DONE, TRAJ_RUNNING, TRAJ_ERROR} traj_status_t;

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
class WamControllerDriverNode : public iri_base_driver::IriBaseNodeDriver<WamControllerDriver>
{
  private:
    // [publisher attributes]
    ros::Publisher libbarrett_link_tcp_publisher_;
    geometry_msgs::PoseStamped PoseStamped_msg_;
    ros::Publisher joint_states_publisher_;
    sensor_msgs::JointState JointState_msg_;

    // [subscriber attributes]
    ros::Subscriber DMPTrackerNewGoal_subscriber_;
    void DMPTrackerNewGoal_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg);
    //CMutex DMPTrackerNewGoal_mutex_;

    // [service attributes]
    ros::ServiceServer hold_on_server_;
    bool hold_onCallback(iri_wam_common_msgs::wamholdon::Request &req, iri_wam_common_msgs::wamholdon::Response &res);
    //CMutex hold_on_mutex_;
    ros::ServiceServer joints_move_server_;
    bool joints_moveCallback(iri_wam_common_msgs::joints_move::Request &req, iri_wam_common_msgs::joints_move::Response &res);
    //CMutex joints_move_mutex_;

    // [client attributes]

    // [action server attributes]
    IriActionServer<iri_wam_common_msgs::DMPTrackerAction> dmp_joint_tracker_aserver_;
    void dmp_joint_trackerStartCallback(const iri_wam_common_msgs::DMPTrackerGoalConstPtr& goal);
    void dmp_joint_trackerStopCallback(void);
    bool dmp_joint_trackerIsFinishedCallback(void);
    bool dmp_joint_trackerHasSucceedCallback(void);
    void dmp_joint_trackerGetResultCallback(iri_wam_common_msgs::DMPTrackerResultPtr& result);
    void dmp_joint_trackerGetFeedbackCallback(iri_wam_common_msgs::DMPTrackerFeedbackPtr& feedback);

    traj_status_t traj_in_joints_status_;
    IriActionServer<control_msgs::FollowJointTrajectoryAction> follow_joint_trajectory_aserver_;
    void follow_joint_trajectoryStartCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
    void follow_joint_trajectoryStopCallback(void);
    bool follow_joint_trajectoryIsFinishedCallback(void);
    bool follow_joint_trajectoryHasSucceedCallback(void);
    void follow_joint_trajectoryGetResultCallback(control_msgs::FollowJointTrajectoryResultPtr& result);
    void follow_joint_trajectoryGetFeedbackCallback(control_msgs::FollowJointTrajectoryFeedbackPtr& feedback);

    // [action client attributes]

   /**
    * \brief post open hook
    * 
    * This function is called by IriBaseNodeDriver::postOpenHook(). In this function
    * specific parameters from the driver must be added so the ROS dynamic 
    * reconfigure application can update them.
    */
    void postNodeOpenHook(void);

  public:
   /**
    * \brief constructor
    *
    * This constructor mainly creates and initializes the WamControllerDriverNode topics
    * through the given public_node_handle object. IriBaseNodeDriver attributes 
    * may be also modified to suit node specifications.
    *
    * All kind of ROS topics (publishers, subscribers, servers or clients) can 
    * be easyly generated with the scripts in the iri_ros_scripts package. Refer
    * to ROS and IRI Wiki pages for more details:
    *
    * http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber(c++)
    * http://www.ros.org/wiki/ROS/Tutorials/WritingServiceClient(c++)
    * http://wikiri.upc.es/index.php/Robotics_Lab
    *
    * \param nh a reference to the node handle object to manage all ROS topics.
    */
    WamControllerDriverNode(ros::NodeHandle& nh);

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~WamControllerDriverNode(void);

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the driver is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

    // [diagnostic functions]

   /**
    * \brief node add diagnostics
    *
    * In this function ROS diagnostics applied to this specific node may be
    * added. Common use diagnostics for all nodes are already called from 
    * IriBaseNodeDriver::addDiagnostics(), which also calls this function. Information
    * of how ROS diagnostics work can be readen here:
    * http://www.ros.org/wiki/diagnostics/
    * http://www.ros.org/doc/api/diagnostic_updater/html/example_8cpp-source.html
    */
    void addNodeDiagnostics(void);

    // [driver test functions]

   /**
    * \brief open status driver tests
    *
    * In this function tests checking driver's functionallity when driver_base 
    * status=open can be added. Common use tests for all nodes are already called
    * from IriBaseNodeDriver tests methods. For more details on how ROS tests work,
    * please refer to the Self Test example in:
    * http://www.ros.org/wiki/self_test/
    */
    void addNodeOpenedTests(void);

   /**
    * \brief stop status driver tests
    *
    * In this function tests checking driver's functionallity when driver_base 
    * status=stop can be added. Common use tests for all nodes are already called
    * from IriBaseNodeDriver tests methods. For more details on how ROS tests work,
    * please refer to the Self Test example in:
    * http://www.ros.org/wiki/self_test/
    */
    void addNodeStoppedTests(void);

   /**
    * \brief run status driver tests
    *
    * In this function tests checking driver's functionallity when driver_base 
    * status=run can be added. Common use tests for all nodes are already called
    * from IriBaseNodeDriver tests methods. For more details on how ROS tests work,
    * please refer to the Self Test example in:
    * http://www.ros.org/wiki/self_test/
    */
    void addNodeRunningTests(void);

   /**
    * \brief specific node dynamic reconfigure
    *
    * This function is called reconfigureHook()
    * 
    * \param level integer
    */
    void reconfigureNodeHook(int level);

};

#endif
