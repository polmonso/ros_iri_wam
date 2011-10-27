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

#ifndef _tibi_dabo_arm_driver_h_
#define _tibi_dabo_arm_driver_h_

#include <iri_base_driver/iri_base_driver.h>
#include <tibi_dabo_arm_node/TibiDaboArmConfig.h>

//include tibi_dabo_arm_driver main library
#include "segway_arm.h"
#include "motion_sequence.h"

extern std::string config_path;
extern std::string motion_path;

/**
 * \brief IRI ROS Specific Driver Class
 *
 * This class inherits from the IRI Base class IriBaseDriver, which provides the
 * guidelines to implement any specific driver. The IriBaseDriver class offers an 
 * easy framework to integrate functional drivers implemented in C++ with the
 * ROS driver structure. ROS driver_base state transitions are already managed
 * by IriBaseDriver.
 *
 * The TibiDaboArmDriver class must implement all specific driver requirements to
 * safetely open, close, run and stop the driver at any time. It also must 
 * guarantee an accessible interface for all driver's parameters.
 *
 * The TibiDaboArmConfig.cfg needs to be filled up with those parameters suitable
 * to be changed dynamically by the ROS dyanmic reconfigure application. The 
 * implementation of the CIriNode class will manage those parameters through
 * methods like postNodeOpenHook() and reconfigureNodeHook().
 *
 */
class TibiDaboArmDriver : public iri_base_driver::IriBaseDriver
{
  private:
    // private attributes and methods
    CSegwayArm arm_driver;
    CMotionSequence *motion_sequence;
    // dynamic reconfigure variables
    std::string arm_config_file;
//    std::string motion_seq_file;
//    double pan_angle,pan_speed;
//    double tilt_angle,tilt_speed; 
    // list of available XML files
    std::vector<std::string> config_files;
    std::vector<std::string> motion_seq_files;
    // configuration variables
    int feedback_rate;
  protected:
    void scan_XML_files(void);
  public:
   /**
    * \brief define config type
    *
    * Define a Config type with the TibiDaboArmConfig. All driver implementations
    * will then use the same variable type Config.
    */
    typedef tibi_dabo_arm_node::TibiDaboArmConfig Config;

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
    TibiDaboArmDriver();

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

    // here define all tibi_dabo_arm_driver interface methods to retrieve and set
    // the driver parameters
    // API for the motion sequences
    /**
     * \brief 
     *
     */
    void start_motion_sequence(std::string &filename);
    /**
     * \brief 
     *
     */
    void start_motion_sequence(std::vector<TMotionStep> &seq);
    /**
     * \brief 
     *
     */
    void pause_motion_sequence(void);
    /**
     * \brief 
     *
     */
    void resume_motion_sequence(void);
    /**
     * \brief 
     *
     */
    void stop_motion_sequence(void);
    /**
     * \brief 
     *
     */
    std::string get_motion_sequence_complete_event_id(void);
    /**
     * \brief 
     *
     */
    std::string get_motion_sequence_error_event_id(void);
    /**
     * \brief 
     *
     */
    std::string get_motion_sequence_error_message(void);
    /**
     * \brief 
     *
     */
    float get_motion_sequence_completed_percentage(void);
    // API for the discete motions
    /**
     * \brief 
     *
     */
    std::string get_position_reached_event_id(void);
    /**
     * \brief functions to move the group of motors in position control
     *
     */
    void move(std::vector<float> &position,std::vector<float> &velocity,std::vector<float> &acceleration);
    /**
     * \brief function to stop all motors
     *
     */
    void stop(void);
    /**
     * \brief function to get the current position of all motors
     *
     */
    void get_position(std::vector<float> &position);
    /**
     * \brief function to get the current velocity of all motors
     *
     */
    void get_velocity(std::vector<float> &velocity);
    /**
     * \brief 
     *
     */
    void set_absolute_motion(bool status);
    /**
     * \brief 
     *
     */
    void set_position_control(bool status);
    // functions to handle the XML files
    /**
     * \brief 
     *
     */
    unsigned int get_num_config_files(void);
    /**
     * \brief 
     *
     */
    std::string get_config_file(unsigned int index);
    /**
     * \brief 
     *
     */
    unsigned int get_num_motion_seq_files(void);
    /**
     * \brief 
     *
     */
    std::string get_motion_seq_file(unsigned int index);
    /**
     * \brief 
     *
     */
    int get_feedback_rate(void);
   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~TibiDaboArmDriver();
};

#endif
