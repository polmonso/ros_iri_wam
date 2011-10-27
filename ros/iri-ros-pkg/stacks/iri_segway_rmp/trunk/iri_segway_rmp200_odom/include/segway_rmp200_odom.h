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

#ifndef _segway_rmp200_odom_h_
#define _segway_rmp200_odom_h_

// [include algorithm common interface]
#include <odometry_core/odometry_iri_base.h>
#include <iri_segway_rmp_msgs/SegwayRMP200Status.h>

// [include generic algorithm class]
#include <generic_odometry_node/generic_odometry.h>

// [include algorithm config file]
#include <iri_segway_rmp200_odom/SegwayRmp200OdomConfig.h>

/**
 * \brief Segway RMP200 Odometry Node
 *
 * This classs implements a common algorithm interface (usually ROS defined).
 * All virtual methods will be implemented to provide an specific behavior to
 * the assossiate generic algorithm. Common features such as message exchange
 * will be already implemented in the generic algoritm class.
 *
 * A Config variable must be defined in order to accomplish dynamic reconfigure
 * requirements. Then parameters defined in the cfg config file will accessible
 * in the config_update method, called whenever there is an update.
 */
class SegwayRmp200Odom : public odometry_core::IriBaseOdometry<iri_segway_rmp_msgs::SegwayRMP200Status>
{
  public:
   /**
    * \brief config type definition
    *
    * Definition of the Config object type according to the automatic generated
    * class from the cfg config file.
    */
    typedef iri_segway_rmp200_odom::SegwayRmp200OdomConfig Config;

   /**
    * \brief config object
    *
    * Instantation of the config variable object.
    */
    Config config_;
    
  protected:
   /**
    * \brief accumulated theta
    *
    * Robot accumulated theta (heading) in radians since odometry beggining.
    */
    double accum_th_;

  public:
   /**
    * \brief constructor
    *
    * In this constructor library objects must be initialized if required.
    */
    SegwayRmp200Odom(void);

   /**
    * \brief destructor
    *
    * The destructor must assert proper closing of the library object.
    */
    ~SegwayRmp200Odom(void);

   /**
    * \brief config update
    *
    * In this function the driver parameters must be updated with the input
    * config variable. Then the new configuration state will be stored in the 
    * Config attribute.
    *
    * \param new_cfg the new driver configuration state
    *
    * \param level level in which the update is taken place
    */
    void config_update(const Config& new_cfg, uint32_t level=0);

   /**
    * \brief compute odometry
    *
    * This function receives a segway rmp200 status message with all available
    * raw data and computes necessary variables to fill up a ROS standard 
    * odometry message.
    *
    * \param msg SegwayRMP200Status segway rmp200 status output message
    */
    void computeOdometry(const iri_segway_rmp_msgs::SegwayRMP200Status & msg);
};

#endif
