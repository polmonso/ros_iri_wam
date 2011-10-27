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

#ifndef _compass_odom_alg_node_h_
#define _compass_odom_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "compass_odom_alg.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// [publisher subscriber headers]
#include <nav_msgs/Odometry.h>
#include <iri_common_drivers_msgs/compass3axis.h>
#include <sensor_msgs/Imu.h>

#include "eventserver.h"
#include "eventexceptions.h"

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class CompassOdomAlgNode : public algorithm_base::IriBaseAlgorithm<CompassOdomAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher compass_odom_publisher_;
    nav_msgs::Odometry Odometry_msg_;
    geometry_msgs::TransformStamped Transform_msg_;
    ros::Publisher compass_imu_publisher_;
    sensor_msgs::Imu Imu_msg_;
    bool publish_imu_;

    // [subscriber attributes]
    ros::Subscriber compass_subscriber_;
    void compass_callback(const iri_common_drivers_msgs::compass3axis & msg);
    CMutex compass_mutex_;

    // event attributes
    CEventServer *ES;       //!< event server
    std::string   TCM3_IN;  //!< unique name of the event
    std::list<std::string> SUBS_LIST;
    bool compass_arrived_;
    std::string parent_id_;
    bool publish_tf_;

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

  public:
   /**
    * \brief Constructor
    *
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    CompassOdomAlgNode(void);

   /**
    * \brief Destructor
    *
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~CompassOdomAlgNode(void);

  protected:

    /**
    * \brief odometry tf broadcaster
    *
    * This attribute is used to broadcast odometry based robot position through
    * tf transforms.
    */
    tf::TransformBroadcaster compass_odom_broadcaster_;

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
