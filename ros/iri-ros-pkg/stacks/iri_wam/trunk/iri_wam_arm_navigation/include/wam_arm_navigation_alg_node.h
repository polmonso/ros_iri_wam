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

#ifndef _wam_arm_navigation_alg_node_h_
#define _wam_arm_navigation_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "wam_arm_navigation_alg.h"

// [publisher subscriber headers]
#include <iri_wam_arm_navigation/PoseSimple.h>

// [service client headers]

// [action server client headers]
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <iri_action_server/iri_action_server.h>
#include <iri_wam_arm_navigation/SimplePoseAction.h>

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class WamArmNavigationAlgNode : public algorithm_base::IriBaseAlgorithm<WamArmNavigationAlgorithm>
{
  private:
    // [publisher attributes]

    // [subscriber attributes]
    ros::Subscriber simple_pose_topic_subscriber_;
    void simple_pose_topic_callback(const iri_wam_arm_navigation::PoseSimple::ConstPtr& msg);
 
    bool aborted;
    CMutex simple_pose_topic_mutex_;

    // [service attributes]

    // [client attributes]

    // [action server attributes]
    IriActionServer<iri_wam_arm_navigation::SimplePoseAction> simple_pose_move_aserver_;
    void simple_pose_moveStartCallback(const iri_wam_arm_navigation::SimplePoseGoalConstPtr& goal);
    void simple_pose_moveStopCallback(void);
    bool simple_pose_moveIsFinishedCallback(void);
    bool simple_pose_moveHasSucceedCallback(void);
    void simple_pose_moveGetResultCallback(iri_wam_arm_navigation::SimplePoseResultPtr& result);
    void simple_pose_moveGetFeedbackCallback(iri_wam_arm_navigation::SimplePoseFeedbackPtr& feedback);

    // [action client attributes]
    actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_iri_wam_client_;
    arm_navigation_msgs::MoveArmGoal move_iri_wam_goal_;
    void move_iri_wamMakeActionRequest(const arm_navigation_msgs::MoveArmGoal& msg);
    void move_iri_wamDone(const actionlib::SimpleClientGoalState& state,  const arm_navigation_msgs::MoveArmResultConstPtr& result);
    void move_iri_wamActive();
    void move_iri_wamFeedback(const arm_navigation_msgs::MoveArmFeedbackConstPtr& feedback);

    arm_navigation_msgs::ArmNavigationErrorCodes code_result;

    std::string state_msg,state__;
    bool END;
  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    WamArmNavigationAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~WamArmNavigationAlgNode(void);
    
     void makeMoveMsg(arm_navigation_msgs::MoveArmGoal& goal);

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
