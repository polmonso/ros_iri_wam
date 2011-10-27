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

#ifndef _covariance_markers_alg_h_
#define _covariance_markers_alg_h_

#include <iri_covariance_markers/CovarianceMarkersConfig.h>
#include "mutex.h"
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

//include covariance_markers_alg main library
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>

/**
 * \brief IRI ROS Specific Driver Class
 *
 *
 */
class CovarianceMarkersAlgorithm
{
  protected:
   /**
    * \brief define config type
    *
    * Define a Config type with the CovarianceMarkersConfig. All driver implementations
    * will then use the same variable type Config.
    */
    CMutex alg_mutex_;

    // private attributes and methods

  public:
   /**
    * \brief define config type
    *
    * Define a Config type with the CovarianceMarkersConfig. All driver implementations
    * will then use the same variable type Config.
    */
    typedef iri_covariance_markers::CovarianceMarkersConfig Config;

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;

   /**
    * \brief constructor
    *
    * In this constructor parameters related to the specific driver can be
    * initalized. Those parameters can be also set in the openDriver() function.
    * Attributes from the main node driver class IriBaseDriver such as loop_rate,
    * may be also overload here.
    */
    CovarianceMarkersAlgorithm();

   /**
    * \brief Lock Algorithm
    *
    * Locks access to the Algorithm class
    */
    void lock(void) { alg_mutex_.enter(); };

   /**
    * \brief Unlock Algorithm
    *
    * Unlocks access to the Algorithm class
    */
    void unlock(void) { alg_mutex_.exit(); };

   /**
    * \brief Tries Access to Algorithm
    *
    * Tries access to Algorithm
    *
    * \return true if the lock was adquired, false otherwise
    */
    bool try_enter(void) { return alg_mutex_.try_enter(); };

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

    // here define all covariance_markers_alg interface methods to retrieve and set
    // the driver parameters

    uint counter_;
    nav_msgs::Odometry odom_;
    visualization_msgs::MarkerArray marker_array_;
    visualization_msgs::Marker current_marker_;

    /** \brief fill marker array message  **/
    void drawCovariance();
    /** \brief get marker array  **/
    visualization_msgs::MarkerArray marker_array();

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~CovarianceMarkersAlgorithm();
};

#endif
