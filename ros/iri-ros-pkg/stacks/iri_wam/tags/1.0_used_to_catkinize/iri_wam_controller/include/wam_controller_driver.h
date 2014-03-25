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

#ifndef _wam_controller_driver_h_
#define _wam_controller_driver_h_

#include <iri_base_driver/iri_base_driver.h>
#include <iri_wam_controller/WamControllerConfig.h>
#include <trajectory_msgs/JointTrajectory.h> 

//include wam_controller_driver main library
#include "wamdriver.h"

const bool HOLDON(true);
const bool HOLDOFF(false);

/**
 * \irief IRI ROS Specific Driver Class
 *
 * This class inherits from the IRI Base class IriBaseDriver, which provides the
 * guidelines to implement any specific driver. The IriBaseDriver class offers an 
 * easy framework to integrate functional drivers implemented in C++ with the
 * ROS driver structure. ROS driver_base state transitions are already managed
 * by IriBaseDriver.
 *
 * The WamControllerDriver class must implement all specific driver requirements to
 * safetely open, close, run and stop the driver at any time. It also must 
 * guarantee an accessible interface for all driver's parameters.
 *
 * The WamControllerConfig.cfg needs to be filled up with those parameters suitable
 * to be changed dynamically by the ROS dyanmic reconfigure application. The 
 * implementation of the CIriNode class will manage those parameters through
 * methods like postNodeOpenHook() and reconfigureNodeHook().
 *
 */
class WamControllerDriver : public iri_base_driver::IriBaseDriver
{
  private:
    // private attributes and methods
    std::string robot_name_; ///< Robot name is needed to name the robot links
    wamDriver *wam_; ///< The low level robot
    trajectory_msgs::JointTrajectoryPoint desired_joint_trajectory_point_;  
  public:
   /**
    * \brief define config type
    *
    * Define a Config type with the WamControllerConfig. All driver implementations
    * will then use the same variable type Config.
    */
    typedef iri_wam_controller::WamControllerConfig Config;

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
    WamControllerDriver(void);

   /**
    * \brief open driver
    *
    * In this function, the driver must be openned. Openning errors must be
    * taken into account. This function is automatically called by 
    * IriBaseDriver::doOpen(), an state transition is performed if return value
    * equals true.
    *
    * \return bool successful
    */
    bool openDriver(void);

   /**
    * \brief close driver
    *
    * In this function, the driver must be closed. Variables related to the
    * driver state must also be taken into account. This function is automatically
    * called by IriBaseDriver::doClose(), an state transition is performed if 
    * return value equals true.
    *
    * \return bool successful
    */
    bool closeDriver(void);

   /**
    * \brief start driver
    *
    * After this function, the driver and its thread will be started. The driver
    * and related variables should be properly setup. This function is 
    * automatically called by IriBaseDriver::doStart(), an state transition is  
    * performed if return value equals true.
    *
    * \return bool successful
    */
    bool startDriver(void);

   /**
    * \brief stop driver
    *
    * After this function, the driver's thread will stop its execution. The driver
    * and related variables should be properly setup. This function is 
    * automatically called by IriBaseDriver::doStop(), an state transition is  
    * performed if return value equals true.
    *
    * \return bool successful
    */
    bool stopDriver(void);

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

    // here define all wam_controller_driver interface methods to retrieve and set
    // the driver parameters

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~WamControllerDriver(void);
    
    
    void get_pose(std::vector<double> *pose);
    void get_joint_angles(std::vector<double> *angles);

    std::string get_robot_name(); 
    inline size_t get_num_joints() {return NJOINTS;};
    /**
     * Checks if the lenght of the vector is correct and if NaN are present
     */
    bool is_joint_vector_valid(const std::vector<double> & angles);

    /**
     * \brief check if the wam is moving right now after a moveTo
     */
    bool is_moving();
    bool is_joint_trajectory_result_succeeded();

    /**
     * \brief Returns the current desired joint trajectory point
     *
     */
    trajectory_msgs::JointTrajectoryPoint get_desired_joint_trajectory_point();

    /**
     * \brief Ask the low level driver to perform a trajectory in joints
     *
     * @ trajectory: list of joints, 
                     can include only position or positions/velocities/accelerations
     */
    bool move_trajectory_in_joints(const trajectory_msgs::JointTrajectory &trajectory, const bool &blocking=false, const bool &compliant=false);
    void stop_trajectory_in_joints();
    bool move_in_joints(std::vector<double> *angles, const double &vel=0.5f, const double &accel=0.5f);
    void wait_move_end();
    void hold_on();
    void hold_off();
    void start_dmp_tracker(const std::vector<double> * initial, const std::vector<double> * goal);
    void dmp_tracker_new_goal(const std::vector<double> * new_goal);
    /**
     * after a /move_trajectory_in_joints
     */
    bool is_moving_trajectory();
 };

#endif
