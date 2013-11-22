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

#ifndef _wam_driver_node_h_
#define _wam_driver_node_h_

#include <iri_base_driver/iri_base_driver_node.h>
#include "wam_driver.h"
#include <Eigen/Dense>

// [publisher subscriber headers]
#include <wam_msgs/RTJointPos.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

// [service client headers]
#include "iri_wam_common_msgs/wamdriver.h"
#include "iri_wam_common_msgs/joints_move.h"
#include "iri_wam_common_msgs/pose_move.h"

// [action server client headers]
#include <iri_wam_common_msgs/DMPTrackerAction.h>
#include <iri_wam_common_msgs/LWPRTrajectoryReturningForceEstimationAction.h>
#include <iri_action_server/iri_action_server.h>

// [action server msgs]
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/action_server.h>

#define HOLDON 0
#define HOLDOFF 1

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

class WamDriverNode : public iri_base_driver::IriBaseNodeDriver<WamDriver>
{
  //JointTrajectoryAction(Diamondback & Electric) 	
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> ActionExecutor;
  typedef ActionExecutor::GoalHandle GoalHandle;
  //FollowJoinTrajectoryAction(Electric)
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> ActionExecutorFollow;
  typedef ActionExecutorFollow::GoalHandle GoalHandleFollow;
  
  private:
    // [publisher attributes]
    ros::Publisher joint_states_publisher;
    sensor_msgs::JointState JointState_msg;
    ros::Publisher pose_publisher;
    geometry_msgs::PoseStamped PoseStamped_msg;

    // [subscriber attributes]
    ros::Subscriber jnt_pos_cmd_subscriber_;
    void jnt_pos_cmd_callback(const wam_msgs::RTJointPos::ConstPtr& msg);
    CMutex jnt_pos_cmd_mutex_;
    ros::Subscriber DMPTrackerNewGoal_subscriber_;
    void DMPTrackerNewGoal_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg);
    CMutex DMPTrackerNewGoal_mutex_;

    // [service attributes]
    ros::ServiceServer wam_services_server_;
    bool wam_servicesCallback(iri_wam_common_msgs::wamdriver::Request  & req,
                              iri_wam_common_msgs::wamdriver::Response & res);

    ros::ServiceServer joints_move_server;
    bool joints_moveCallback(iri_wam_common_msgs::joints_move::Request  & req,
                             iri_wam_common_msgs::joints_move::Response & res);

    ros::ServiceServer pose_move_server;
    bool pose_moveCallback(iri_wam_common_msgs::pose_move::Request  & req,
                           iri_wam_common_msgs::pose_move::Response & res);

    // [client attributes]

    // [action server attributes]
    IriActionServer<iri_wam_common_msgs::DMPTrackerAction> DMPTracker_aserver_;
    void DMPTrackerStartCallback(const iri_wam_common_msgs::DMPTrackerGoalConstPtr& goal);
    void DMPTrackerStopCallback(void);
    bool DMPTrackerIsFinishedCallback(void);
    bool DMPTrackerHasSucceedCallback(void);
    void DMPTrackerGetResultCallback(iri_wam_common_msgs::DMPTrackerResultPtr& result);
    void DMPTrackerGetFeedbackCallback(iri_wam_common_msgs::DMPTrackerFeedbackPtr& feedback);
    IriActionServer<iri_wam_common_msgs::LWPRTrajectoryReturningForceEstimationAction> lwpr_trajectory_server_aserver_;
    void lwpr_trajectory_serverStartCallback(const iri_wam_common_msgs::LWPRTrajectoryReturningForceEstimationGoalConstPtr& goal);
    void lwpr_trajectory_serverStopCallback(void);
    bool lwpr_trajectory_serverIsFinishedCallback(void);
    bool lwpr_trajectory_serverHasSucceedCallback(void);
    void lwpr_trajectory_serverGetResultCallback(iri_wam_common_msgs::LWPRTrajectoryReturningForceEstimationResultPtr& result);
    void lwpr_trajectory_serverGetFeedbackCallback(iri_wam_common_msgs::LWPRTrajectoryReturningForceEstimationFeedbackPtr& feedback);

    IriActionServer<control_msgs::FollowJointTrajectoryAction> follow_joint_trajectory_server_;
    void joint_trajectoryStartCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
    void joint_trajectoryStopCallback(void);
    bool joint_trajectoryIsFinishedCallback(void);
    bool joint_trajectoryHasSucceedCallback(void);
    void joint_trajectoryGetResultCallback(control_msgs::FollowJointTrajectoryResultPtr& result);
    void joint_trajectoryGetFeedbackCallback(control_msgs::FollowJointTrajectoryFeedbackPtr& feedback);

    void goalCB(GoalHandle gh);
    void cancelCB(GoalHandle gh);
    
    void goalFollowCB(GoalHandleFollow gh);
    void canceFollowlCB(GoalHandleFollow gh);    
    // [action client attributes]
    
   /**
    * \brief post open hook
    * 
    * This function is called by IriBaseNodeDriver::postOpenHook(). In this function
    * specific parameters from the driver must be added so the ROS dynamic 
    * reconfigure application can update them.
    */
    void postNodeOpenHook(void);
    
    void trajectory2follow(trajectory_msgs::JointTrajectory traj, bool& state);

  public:
   /**
    * \brief constructor
    *
    * This constructor mainly creates and initializes the WamDriverNode topics
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
    WamDriverNode(ros::NodeHandle& nh);

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~WamDriverNode();

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the driver is on running state. Loop frequency 
    * can be tuned my modifying loop_rate attribute.
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
