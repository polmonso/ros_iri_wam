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

#ifndef _iri_joystick_driver_h_
#define _iri_joystick_driver_h_

#include "iri_base_driver/iri_base_driver.h"
#include "iri_joystick/IriJoystickConfig.h"
#include <iridrivers/joystick.h>

//include iri_joystick_driver main library

/**
 * \brief IRI ROS Specific Driver Class
 *
 * This class inherits from the IRI Base class IriBaseDriver, which provides the
 * guidelines to implement any specific driver. The IriBaseDriver class offers an 
 * easy framework to integrate functional drivers implemented in C++ with the
 * ROS driver structure. ROS driver_base state transitions are already managed
 * by IriBaseDriver.
 *
 * The IriJoystickDriver class must implement all specific driver requirements to
 * safetely open, close, run and stop the driver at any time. It also must 
 * guarantee an accessible interface for all driver's parameters.
 *
 * The IriJoystickConfig.cfg needs to be filled up with those parameters suitable
 * to be changed dynamically by the ROS dyanmic reconfigure application. The 
 * implementation of the CIriNode class will manage those parameters through
 * methods like postNodeOpenHook() and reconfigureNodeHook().
 *
 */
class IriJoystickDriver : public iri_base_driver::IriBaseDriver
{
  private:
    /*
    * Pointer to IRI driver class to make the real operation with the·
    * hardware. Default name is iri-joystick-${PID}
    */
    CJoystick * jh_;

   /*
    * linux device path to found the joystick in. Default is /dev/input/js0
    */
    std::string dev_;
   /*
    * Publising topic name. Default is "joy"
    */
    std::string pub_channel_name_;


  public:
   /**
    * \brief define config type
    *
    * Define a Config type with the IriJoystickConfig. All driver implementations
    * will then use the same variable type Config.
    */
    typedef iri_joystick::IriJoystickConfig Config;

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
    IriJoystickDriver();

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
    * \brief start driver
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
    void config_update(const Config& new_cfg, uint32_t level=0);

    void enable_position_change_callback(int axis_id,
                          void (* callback) (void * param, unsigned int axis_id, float value),
                          void * param = 0);

    void enable_button_callback(int button_id,
                          void (* callback) (void * param, unsigned int button_id, bool value),
                          void * param = 0);
    /**
     * return the total number of buttons as int
     */
    int get_num_buttons() const;

    /*
     * return the total number of axes detected in device
     */
    int get_num_axes() const;

   /**
    * return the physical device string from driver
    */
    std::string get_device() const;

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~IriJoystickDriver();
};

#endif
