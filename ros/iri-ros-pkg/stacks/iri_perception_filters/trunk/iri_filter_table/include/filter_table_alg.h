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

#ifndef _filter_table_alg_h_
#define _filter_table_alg_h_

#include <iri_filter_table/FilterTableConfig.h>
#include "mutex.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

//include filter_table_alg main library

/**
 * \brief IRI ROS Specific Driver Class
 *
 *
 */
class FilterTableAlgorithm
{
  protected:
   /**
    * \brief define config type
    *
    * Define a Config type with the FilterTableConfig. All driver implementations
    * will then use the same variable type Config.
    */
    CMutex alg_mutex_;

    // private attributes and methods
    float max_z_; //0.8
    float sac_dist_thr_;  //0.001
    float filter_final_ratio_; //0.6
    float rad_radius_search_; //0.1

  public:
   /**
    * \brief define config type
    *
    * Define a Config type with the FilterTableConfig. All driver implementations
    * will then use the same variable type Config.
    */
    typedef iri_filter_table::FilterTableConfig Config;

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
    FilterTableAlgorithm(void);

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
    void config_update(Config& new_cfg, uint32_t level=0);

    // here define all filter_table_alg interface methods to retrieve and set
    // the driver parameters
    inline float set_max_z(float max_z)
    {
      this->max_z_=max_z;
      return this->max_z_;
    }

    inline float set_sac_dist_thr(float sac_dist_thr)
    {
      this->sac_dist_thr_=sac_dist_thr;
      return this->sac_dist_thr_;
    }

    inline float set_filter_final_ratio(float filter_final_ratio)
    {
      this->filter_final_ratio_=filter_final_ratio;
      return this->filter_final_ratio_;
    }

    inline float set_rad_radius_search(float rad_radius_search)
    {
      this->rad_radius_search_=rad_radius_search;
      return this->rad_radius_search_;
    }

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~FilterTableAlgorithm(void);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_table(const sensor_msgs::PointCloud2::ConstPtr &cloud_blob);

};


#endif
