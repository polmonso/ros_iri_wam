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

#ifndef _hrengagement_alg_node_h_
#define _hrengagement_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "hrengagement_alg.h"

// [publisher subscriber headers]
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <iri_people_tracking/peopleTrackingArray.h>

// [service client headers]

// [action server client headers]
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tibi_dabo_msgs/sequenceAction.h>

#include <tf/transform_listener.h>

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class HrengagementAlgNode : public algorithm_base::IriBaseAlgorithm<HrengagementAlgorithm>
{
  private:
    // [publisher attributes]

    // [subscriber attributes]
    ros::Subscriber people_tracker_subscriber_;
    void people_tracker_callback(const iri_people_tracking::peopleTrackingArray::ConstPtr& msg);
    CMutex people_tracker_mutex_;

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]
    actionlib::SimpleActionClient<tibi_dabo_msgs::sequenceAction> hri_client_;
    tibi_dabo_msgs::sequenceGoal hri_goal_;
    void HRIMakeActionRequest(const std::vector<std::string> & xml_files);
    void HRIDone(const actionlib::SimpleClientGoalState& state,  const tibi_dabo_msgs::sequenceResultConstPtr& result);
    void HRIActive(void);
    void HRIFeedback(const tibi_dabo_msgs::sequenceFeedbackConstPtr& feedback);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBase_client_;
    void MoveBaseMakeActionRequest(const geometry_msgs::PoseStamped & new_goal);
    void MoveBaseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result);
    void MoveBaseActive(void);
    void MoveBaseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    iri_people_tracking::peopleTracking goal_person_;
    tf::TransformListener tf_listener_;
    bool track_people_;
    
    typedef enum {TTS_=0,LEDS_=1,HEAD_=2,LEFT_ARM_=3,RIGHT_ARM_=4} clients;
    static const int NUM_CLIENTS=RIGHT_ARM_-TTS_+1;

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    HrengagementAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~HrengagementAlgNode(void);

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
