// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author Sergi Hernandez & Joan Perez
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

#ifndef _segway_rmp200_driver_h_
#define _segway_rmp200_driver_h_

#include <iri_base_driver/iri_base_driver.h>
#include <iri_segway_rmp200/SegwayRmp200Config.h>
#include "exceptions.h"

//include segway_rmp200_driver main library
#include "segway_rmp200.h"
#include "segway_rmp200_status.h"

namespace segway_rmp200_node
{

/**
 * \brief IRI ROS Specific Driver Class
 *
 * This class inherits from the IRI Base class IriBaseDriver, which provides the
 * guidelines to implement any specific driver. The IriBaseDriver class offers an 
 * easy framework to integrate functional drivers implemented in C++ with the
 * ROS driver structure. ROS driver_base state transitions are already managed
 * by IriBaseDriver.
 *
 * The SegwayRmp200Driver class must implement all specific driver requirements to
 * safetely open, close, run and stop the driver at any time. It also must 
 * guarantee an accessible interface for all driver's parameters.
 *
 * The SegwayRmp200Config.cfg needs to be filled up with those parameters suitable
 * to be changed dynamically by the ROS dyanmic reconfigure application. The 
 * implementation of the CIriNode class will manage those parameters through
 * methods like postNodeOpenHook() and reconfigureNodeHook().
 *
 */
class SegwayRmp200Driver : public iri_base_driver::IriBaseDriver
{
  private:
    // private attributes and methods
    CSegwayRMP200 *segway_;
    CFTDIServer *ftdi_server_;
    std::string serial_number_;
    std::list<std::string> events_;

  public:
    enum segway_events_enum {NO_USB=0, POWER_OFF, NO_HEART_BEAT, NEW_STATUS};

   /**
    * \brief define config type
    *
    * Define a Config type with the SegwayRmp200Config. All driver implementations
    * will then use the same variable type Config.
    */
    typedef iri_segway_rmp200::SegwayRmp200Config Config;

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
    SegwayRmp200Driver();

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

    // here define all segway_rmp200_driver interface methods to retrieve and set
    // the driver parameters
    iri_segway_rmp_msgs::SegwayRMP200Status get_status(void);
    float get_pitch_angle(void);
    float get_pitch_rate(void);
    float get_roll_angle(void);
    float get_roll_rate(void);
    float get_left_wheel_velocity(void);
    float get_right_wheel_velocity(void);
    float get_yaw_rate(void);
    float get_uptime(void);
    float get_left_wheel_displacement(void);
    float get_right_wheel_displacement(void);
    float get_forward_displacement(void);
    float get_yaw_displacement(void);
    float get_left_motor_torque(void);
    float get_right_motor_torque(void);
    op_mode get_operation_mode(void);
    gain get_gain_schedule(void);
    float get_ui_battery_voltage(void);
    float get_powerbase_battery_voltage(void);
    void reset(void);
    void move_platform(float vT, float vR);
    void stop_platform(void);
    const std::list<std::string> & getSegwayEvents() const;
    void setSegwayEvents(void);
    static const std::string & getSegwayEventName(const unsigned int & ev);

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~SegwayRmp200Driver();
};

} // end of namespace

#endif
