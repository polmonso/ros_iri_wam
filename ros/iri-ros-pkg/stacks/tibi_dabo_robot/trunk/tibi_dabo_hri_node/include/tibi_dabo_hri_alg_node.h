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

#ifndef _tibi_dabo_hri_alg_node_h_
#define _tibi_dabo_hri_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "tibi_dabo_hri_alg.h"

// [publisher subscriber headers]
#include <std_msgs/String.h>

// [service client headers]

// [action server client headers]
#include <iri_action_server/iri_action_server.h>
#include <tibi_dabo_msgs/sequenceAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iri_common_drivers_msgs/ttsAction.h>

typedef enum {TTS_=0,LEDS_=1,HEAD_=2,LEFT_ARM_=3,RIGHT_ARM_=4} clients;

typedef actionlib::SimpleActionClient<tibi_dabo_msgs::sequenceAction> SeqClient;

typedef struct 
{
  SeqClient *client;
  bool succeeded;
  bool active;
  bool loaded;
  float percentage;
}TSeqClient;

typedef actionlib::SimpleActionClient<iri_common_drivers_msgs::ttsAction> TtsClient;

typedef struct 
{
  TtsClient *client;
  bool succeeded;
  bool active;
  bool loaded;
  float percentage;
}TTtsClient;
/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class TibiDaboHriAlgNode : public algorithm_base::IriBaseAlgorithm<TibiDaboHriAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher facial_expression_publisher_;

    // [subscriber attributes]

    // [service attributes]

    // [client attributes]

    // [action server attributes]
    IriActionServer<tibi_dabo_msgs::sequenceAction> seqs_aserver_;
    void startCallback(const tibi_dabo_msgs::sequenceGoalConstPtr& goal);
    void stopCallback(void);
    bool isFinishedCallback(void);
    bool hasSucceedCallback(void);
    void getResultCallback(tibi_dabo_msgs::sequenceResultPtr& result);
    void getFeedbackCallback(tibi_dabo_msgs::sequenceFeedbackPtr& feedback);

    // [action client attributes]
    // Loquendo TTS
    TTtsClient tts_;
    void ttsMakeActionRequest(const iri_common_drivers_msgs::ttsGoal & tts_goal);
    void ttsDone(const actionlib::SimpleClientGoalState& state, const iri_common_drivers_msgs::ttsResultConstPtr& result);
    void ttsActive(void);
    void ttsFeedback(const iri_common_drivers_msgs::ttsFeedbackConstPtr& feedback);

    // Head Leds
    TSeqClient leds_;
    void ledsMakeActionRequest(const tibi_dabo_msgs::sequenceGoal & leds_goal);
    void ledsDone(const actionlib::SimpleClientGoalState& state, const tibi_dabo_msgs::sequenceResultConstPtr& result);
    void ledsActive(void);
    void ledsFeedback(const tibi_dabo_msgs::sequenceFeedbackConstPtr& feedback);
    
    // Head sequence
    TSeqClient head_;
    void headMakeActionRequest(const tibi_dabo_msgs::sequenceGoal & head_goal);
    void headDone(const actionlib::SimpleClientGoalState& state, const tibi_dabo_msgs::sequenceResultConstPtr& result);
    void headActive(void);
    void headFeedback(const tibi_dabo_msgs::sequenceFeedbackConstPtr& feedback);
    
    // left arm sequence
    TSeqClient left_arm_;
    void leftArmMakeActionRequest(const tibi_dabo_msgs::sequenceGoal & left_arm_goal);
    void leftArmDone(const actionlib::SimpleClientGoalState& state, const tibi_dabo_msgs::sequenceResultConstPtr& result);
    void leftArmActive(void);
    void leftArmFeedback(const tibi_dabo_msgs::sequenceFeedbackConstPtr& feedback);

    // right arm sequence
    TSeqClient right_arm_;
    void rightArmMakeActionRequest(const tibi_dabo_msgs::sequenceGoal & right_arm_goal);
    void rightArmDone(const actionlib::SimpleClientGoalState& state, const tibi_dabo_msgs::sequenceResultConstPtr& result);
    void rightArmActive(void);
    void rightArmFeedback(const tibi_dabo_msgs::sequenceFeedbackConstPtr& feedback);
    

    bool empty_leds_sequence_;
  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    TibiDaboHriAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~TibiDaboHriAlgNode(void);

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
    
    void cancelCurrentGoals(void);
    void speak(void);
};

#endif
