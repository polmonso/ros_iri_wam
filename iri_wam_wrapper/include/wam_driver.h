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

#ifndef _wam_driver_h_
#define _wam_driver_h_

#include <iri_base_driver/iri_base_driver.h>
#include <iri_wam_wrapper/WamConfig.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>

//include wam_driver main library
//#include "CWamDriver.h"
//#include "wamdriver.h"

struct ForceRequest
{
    enum Tstatus {
        QUIET,
        ONGOING,
        FAILED,
        SUCCESS
    };

    Tstatus status;
    double force_value;

    void init()
    {
        status      = QUIET;
        force_value = 0.0;
    }

    void success_response(double v)
    {
        status      = SUCCESS;
        force_value = v;
    }

    bool is_estimate_force_request_finish()
    {
        return (status != ONGOING);
    }

    bool was_estimate_force_request_succedded()
    {
        return (status == SUCCESS);
    }
};

/**
 * \brief IRI ROS Specific Driver Class
 *
 * This class inherits from the IRI Base class IriBaseDriver, which provides the
 * guidelines to implement any specific driver. The IriBaseDriver class offers an
 * easy framework to integrate functional drivers implemented in C++ with the
 * ROS driver structure. ROS driver_base state transitions are already managed
 * by IriBaseDriver.
 *
 * The WamDriver class must implement all specific driver requirements to
 * safetely open, close, run and stop the driver at any time. It also must
 * guarantee an accessible interface for all driver's parameters.
 *
 * The WamConfig.cfg needs to be filled up with those parameters suitable
 * to be changed dynamically by the ROS dyanmic reconfigure application. The
 * implementation of the CIriNode class will manage those parameters through
 * methods like postNodeOpenHook() and reconfigureNodeHook().
 *
 */
class WamDriver : public iri_base_driver::IriBaseDriver
{
  private:
    // private attributes and methods
    std::string robot_name_;
    std::string wamserver_ip;
    int server_port;
    int state_refresh_rate;
    wamDriver *wam_;
    /**
      * Object for handling force estimation request process
      */
    boost::shared_ptr<ForceRequest> force_request_;

    trajectory_msgs::JointTrajectoryPoint desired_joint_trajectory_point_;

    /**
     * \brief check if move in joints request is sane
     *
     * Control if the vector has the same size of robot joints and
     * if there is no Nan values
     */
    bool is_joints_move_request_valid(const std::vector<double> & angles);

  public:
   /**
    * \brief define config type
    *
    * Define a Config type with the WamConfig. All driver implementations
    * will then use the same variable type Config.
    */
    typedef iri_wam_wrapper::WamConfig Config;

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
    WamDriver();

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
    void config_update(const Config& new_cfg, uint32_t level=0);

    // here define all wam_driver interface methods to retrieve and set
    // the driver parameters

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~WamDriver();
    std::string get_robot_name();
    int get_num_joints();

    /**
     * \brief check if the wam is moving right now
     */
    bool is_moving();
    bool is_joint_trajectory_result_succeeded();
    void wait_move_end();
    void get_pose(std::vector<double> *pose);
    void get_joint_angles(std::vector<double> *angles);
    void move_in_joints(std::vector<double> *angles, std::vector<double>* vels = NULL, std::vector<double>* accs = NULL);
    void hold_current_position(bool on);
    /**
     * \brief Ask the low level driver to perform a movement to reach cartesian pose
     *
     * This funcion will translate from geometry_pose to low level format and
     * call the low level driver function to perform the cartesian move
     */
    void move_in_cartesian_pose(const geometry_msgs::Pose pose, const double vel = 0, const double acc = 0);

    /**
     * \brief Returns the current desired joint trajectory point
     *
     */
    trajectory_msgs::JointTrajectoryPoint get_desired_joint_trajectory_point();

    /**
     * \brief Ask the low level driver to perform a trajectory in joints
     *
     * This functions will translate from joint trajectory message to low level
     * driver types and sent the command to perform the trajectory given by
     * the joint positions.
     */
    void move_trajectory_in_joints(const trajectory_msgs::JointTrajectory & trajectory);

    void stop_trajectory_in_joints();

    /**
     * \brief Ask the low level driver to perform a LWPR trajectory and return
     * force estimation
     *
     * This function will send to the low level driver the request to use the
     * files which contains an LWPR model and trajectory points. It will perform
     * the trajectory and return the estimate force at the end
     *
     * \param model_filename: full server system path which contains the LWPR
     * model
     * \param points_filename: full server system path which contains the points
     * for the trajectory
     */
    void move_trajectory_learnt_and_estimate_force(const std::string model_filename,
                                                   const std::string points_filename);

    /**
     * \brief return a reference to current force request info
     */
    const boost::shared_ptr<ForceRequest> get_force_request_info()
    {
        return force_request_;
    }

    /**
     * \brief Starts the DMP tracker
     */
    void start_dmp_tracker(const std::vector<double> * initial, const std::vector<double> * goal);

    void dmp_tracker_new_goal(const std::vector<double> * new_goal);

    void jnt_pos_cmd_callback(const std::vector<float> * joints,
                              const std::vector<float> * rate_limits);
};

#endif
