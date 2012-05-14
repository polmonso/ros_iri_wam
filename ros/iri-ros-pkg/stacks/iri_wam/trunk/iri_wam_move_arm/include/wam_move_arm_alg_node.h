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

#ifndef _wam_move_arm_alg_node_h_
#define _wam_move_arm_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "wam_move_arm_alg.h"
#include <trajectory_msgs/JointTrajectory.h>

// [publisher subscriber headers]

// [service client headers]

// [action server client headers]
#include <iri_action_server/iri_action_server.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class WamMoveArmAlgNode : public algorithm_base::IriBaseAlgorithm<WamMoveArmAlgorithm>
{
  private:
    // [publisher attributes]

    // [subscriber attributes]

    // [service attributes]

    // [client attributes]

    // [action server attributes]
    IriActionServer<control_msgs::FollowJointTrajectoryAction> syn_controller_aserver_;
    void syn_controllerStartCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
    void syn_controllerStopCallback(void);
    bool syn_controllerIsFinishedCallback(void);
    bool syn_controllerHasSucceedCallback(void);
    void syn_controllerGetResultCallback(control_msgs::FollowJointTrajectoryResultPtr& result);
    void syn_controllerGetFeedbackCallback(control_msgs::FollowJointTrajectoryFeedbackPtr& feedback);
    bool finish;
    bool success_contr;
   
    IriActionServer<arm_navigation_msgs::MoveArmAction> move_arm_aserver_;
    void move_armStartCallback(const arm_navigation_msgs::MoveArmGoalConstPtr& goal);
    void move_armStopCallback(void);
    bool move_armIsFinishedCallback(void);
    bool move_armHasSucceedCallback(void);
    void move_armGetResultCallback(arm_navigation_msgs::MoveArmResultPtr& result);
    void move_armGetFeedbackCallback(arm_navigation_msgs::MoveArmFeedbackPtr& feedback);


    // [action client attributes]
    actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> syn_move_arm_client_;
    arm_navigation_msgs::MoveArmGoal syn_move_arm_goal_;
    void syn_move_armMakeActionRequest(const arm_navigation_msgs::MoveArmGoal& goal);
    void syn_move_armDone(const actionlib::SimpleClientGoalState& state,  const arm_navigation_msgs::MoveArmResultConstPtr& result);
    void syn_move_armActive();
    void syn_move_armFeedback(const arm_navigation_msgs::MoveArmFeedbackConstPtr& feedback);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> controller_client_;
    control_msgs::FollowJointTrajectoryGoal cliente_goal_;
    void clienteMakeActionRequest();
    void clienteDone(const actionlib::SimpleClientGoalState& state,  const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    void clienteActive();
    void clienteFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);



	bool send_msg_;
	control_msgs::FollowJointTrajectoryGoal tmp_msg_;
	arm_navigation_msgs::MoveArmResult move_result;
	arm_navigation_msgs::MoveArmFeedback move_feedback;
	actionlib::SimpleClientGoalState *goal_state_;
	void makeMsg(control_msgs::FollowJointTrajectoryGoal& msg);
	void move_arm(const arm_navigation_msgs::MoveArmGoalConstPtr& goal);
	void getTime(const arm_navigation_msgs::MoveArmGoal& msg, ros::Duration& time);
	
	control_msgs::FollowJointTrajectoryFeedback feedback_controller;
	control_msgs::FollowJointTrajectoryResult	result_controller;

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    WamMoveArmAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~WamMoveArmAlgNode(void);

  protected:
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

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
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

#endif
