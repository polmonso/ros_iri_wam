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

#ifndef _IRI_BASE_ALGORITHM_H
#define _IRI_BASE_ALGORITHM_H

#include <ros/ros.h>
#include <signal.h>

// iri-utils thread server
#include "threadserver.h"
#include "exceptions.h"
#include "mutex.h"

// boost thread includes for ROS::spin thread
#include <boost/thread.hpp>
#include <boost/bind.hpp>

// dynamic reconfigure server include
#include <dynamic_reconfigure/server.h>

// diagnostic updater include
#include <diagnostic_updater/diagnostic_updater.h>

namespace algorithm_base
{

/**
 * \brief Abstract Class for Ctrl+C management
 *
 */
class AbstractAlgorithmNode
{
  protected:
    static int ctrl_c_hit_count_;
    
    static void hupCalled(int sig)
    {
      ROS_WARN("Unexpected SIGHUP caught. Ignoring it.");
    }
    
    static void sigCalled(int sig)
    {
      ctrl_c_hit_count_++;
    }
};

/**
 * \brief IRI ROS Algorithm Base Node Class
 *
 * This class provides a common framework for all kind of algorithms, similar to 
 * the one defined for the ROS driver_base::DriverNode<TemplateDriver> for the
 * driver environment. In this case, the template Algorithm class must be an
 * implementation derivated from a common interface (preferably ROS defined).
 * The inherit template design form allows complete access to the generic
 * algorithm object while manteining flexibility to instantiate any object which
 * inherits from it.
 *
 * An abstract class is generated as a ROS package to force implementation of
 * virtual methods in the generic algorithm layer. Functions to perform tests,
 * add diagnostics or dynamically reconfigure the algorithm parameters are
 * offered respecting the ROS driver_base terminology. Similarly to the
 * IriBaseNodeDriver common purpose behaviours for all algorithm kinds can be
 * defined and forwarded to each of the generic algorithm classes for specific
 * operations.
 *
 * In addition, a mainThread is provided like in the IriBaseNodeDriver class.
 * Threads are implemented using iri_utils software utilities. The mainThread() 
 * function loops in a defined loop_rate_. In each iteration, the abstract 
 * mainNodeThread() function defined in the inherit node class is called.
 */
template <class Algorithm>
class IriBaseAlgorithm : public AbstractAlgorithmNode
{
  public:
   /**
    * \brief config object
    *
    * All template Algorithm class will need a Config variable to allow ROS
    * dynamic reconfigure. This config class will contain all algorithm 
    * parameters which may be modified once the algorithm node is launched.
    */
    typedef typename Algorithm::Config Config;

  protected:
   /**
    * \brief Reference to the unique thread handler
    *
    * This reference to the unique thread handler is initialized when an object 
    * of this class is first created. It is used to create and handle all the
    * IriBaseNodeDriver threads. The object pointed by this reference is shared 
    * by all objects in any application.
    */
    CThreadServer *thread_server_;

   /**
    * \brief identifier of the node main thread
    *
    * This string has the identifier of the main IriBaseNodeDriver thread.  
    * This string is initialized at contruction time. This thread is only used 
    * internally to the class, so it is not possible to get its identifier out.
    */
    std::string main_thread_id_;

   /**
    * \brief template algorithm class
    *
    * This template class refers to an implementation of an specific algorithm
    * interface. Will be used in the derivate class to define the common 
    * behaviour for all the different implementations from the same algorithm.
    */
    Algorithm alg_;

   /**
    * \brief public node handle communication object
    *
    * This node handle is going to be used to create topics and services within
    * the node namespace. Additional node handles can be instantatied if 
    * additional namespaces are needed.
    */
    ros::NodeHandle public_node_handle_;

   /**
    * \brief private node handle object
    *
    * This private node handle will be used to define algorithm parameters into
    * the ROS parametre server. For communication pruposes please use the 
    * previously defined node_handle_ object.
    */
    ros::NodeHandle private_node_handle_;

   /**
    * \brief main thread loop rate
    * 
    * This value determines the loop frequency of the node main thread function
    * mainThread() in HZ. It is initialized at construction time. This variable 
    * may be modified in the node implementation constructor if a desired
    * frequency is required.
    */
    ros::Rate loop_rate_;

   /**
    * \brief default main thread frequency
    * 
    * This constant determines the default frequency of the mainThread() in HZ.
    * All nodes will loop at this rate if loop_rate_ variable is not modified.
    */
    static const unsigned int DEFAULT_RATE = 10; //[Hz]
    
   /**
    * \brief diagnostic updater
    * 
    * The diagnostic updater allows definition of custom diagnostics. 
    * 
    */
    diagnostic_updater::Updater diagnostic_;

  public:
   /**
    * \brief constructor
    * 
    * This constructor initializes all the node handle objects, the main thread
    * loop_rate_ and the diagnostic updater. It also instantiates the main 
    * thread of the class and the dynamic reconfigure callback.
    */
    IriBaseAlgorithm(void);

   /**
    * \brief destructor
    * 
    * This destructor kills the main thread.
    */
    ~IriBaseAlgorithm(void);

   /**
    * \brief spin
    * 
    * This method is meant to spin the node to update all ROS features. It also
    * launches de main thread. Once the object is instantiated, it will not 
    * start iterating until this method is called.
    */
    int spin(void);

  private:
   /**
    * \brief ros spin thread
    * 
    * This boost thread object will manage the ros::spin function. It is 
    * instantiated and launched in the spin class method.
    */
    boost::shared_ptr<boost::thread> ros_thread_;

   /**
    * \brief dynamic reconfigure server
    * 
    * The dynamic reconfigure server is in charge to retrieve the parameters
    * defined in the config cfg file through the reconfigureCallback.
    */
    dynamic_reconfigure::Server<Config> dsrv_;

  protected:
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
    void reconfigureCallback(Config &config, uint32_t level);

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
    virtual void node_config_update(Config &config, uint32_t level) = 0;
   
   /**
    * \brief add diagnostics
    * 
    * In this function ROS diagnostics applied to all algorithms nodes may be
    * added. It calls the addNodeDiagnostics method.
    */
    void addDiagnostics(void);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    virtual void addNodeDiagnostics(void) = 0;

//     void addOpenedTests(void);
//     void addStoppedTests(void);
//     void addRunningTests(void);
//     virtual void addNodeOpenedTests(void) = 0;
//     virtual void addNodeStoppedTests(void) = 0;
//     virtual void addNodeRunningTests(void) = 0;

   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every inherit algorithm node object. The loop won't exit until the node
    * finish its execution. The commands implemented in the abstract function
    * mainNodeThread() will be executed in every iteration.
    * 
    * Loop frequency can be tuned my modifying loop_rate_ attribute.
    * 
    * \param param is a pointer to a IriBaseAlgorithm object class. It is used
    *              to access to the object attributes and methods.
    */
    static void *mainThread(void *param);

   /**
    * \brief specific node thread
    *
    * In this abstract function specific commands for each algorithm node
    * have to be detailed.
    */
    virtual void mainNodeThread(void) = 0;
};


template <class Algorithm>
IriBaseAlgorithm<Algorithm>::IriBaseAlgorithm() : 
  public_node_handle_(ros::this_node::getName()),
  private_node_handle_("~"), 
  loop_rate_(DEFAULT_RATE),
  diagnostic_(),
  dsrv_(private_node_handle_)
{
  ROS_DEBUG("IriBaseAlgorithm::Constructor");
  // create thread server instance
  this->thread_server_  = CThreadServer::instance();

  // identify main thread
  this->main_thread_id_ = "main_thread";

  // create main thread and attach to server
  this->thread_server_->create_thread(this->main_thread_id_);
  this->thread_server_->attach_thread(this->main_thread_id_, this->mainThread, this);

//   // assign callback to dynamic reconfigure server
//   dsrv_.setCallback(boost::bind(&IriBaseAlgorithm<Algorithm>::reconfigureCallback, this, _1, _2));

}

template <class Algorithm>
IriBaseAlgorithm<Algorithm>::~IriBaseAlgorithm()
{
  ROS_DEBUG("IriBaseAlgorithm::Destructor");

  this->thread_server_->kill_thread(this->main_thread_id_);
}

template <class Algorithm>
void IriBaseAlgorithm<Algorithm>::reconfigureCallback(Config &config, uint32_t level)
{
  ROS_DEBUG("IriBaseAlgorithm::reconfigureCallback");
  this->node_config_update(config, level);
  this->alg_.config_update(config, level);
}

template <class Algorithm>
void IriBaseAlgorithm<Algorithm>::addDiagnostics(void)
{
  ROS_DEBUG("IriBaseAlgorithm::addDiagnostics");
  addNodeDiagnostics();
}

template <class Algorithm>
void *IriBaseAlgorithm<Algorithm>::mainThread(void *param)
{
  ROS_DEBUG("IriBaseAlgorithm::mainThread");

  // retrieve base algorithm class
  IriBaseAlgorithm *iriNode = (IriBaseAlgorithm *)param;

  while(ros::ok() && !ctrl_c_hit_count_)
  {
    // run node stuff
    iriNode->mainNodeThread();

    // sleep remainder time
    iriNode->loop_rate_.sleep();
  }

  // kill main thread
  pthread_exit(NULL);
}

template <class Algorithm>
int IriBaseAlgorithm<Algorithm>::spin(void)
{
  ROS_DEBUG("IriBaseAlgorithm::spin");

  // initialize diagnostics
  this->diagnostic_.setHardwareID("none");
  this->addDiagnostics();
  
  // launch ros spin in different thread
  this->ros_thread_.reset( new boost::thread(boost::bind(&ros::spin)) );
  assert(ros_thread_);

  // assign callback to dynamic reconfigure server
  this->dsrv_.setCallback(boost::bind(&IriBaseAlgorithm<Algorithm>::reconfigureCallback, this, _1, _2));

  // launch node thread
  this->thread_server_->start_thread( this->main_thread_id_);
  
  while(ros::ok() && !ctrl_c_hit_count_)
  {
    // update diagnostics
    this->diagnostic_.update();
    
    ros::WallDuration(0.1).sleep();
  }

  // stop ros
  ros::shutdown();

  // kill ros spin thread
  this->ros_thread_.reset();

  return 0;
}

// definition of the static Ctrl+C counter
int AbstractAlgorithmNode::ctrl_c_hit_count_ = 0;

/**
 * \brief base main
 *
 * This main is common for all the algorithm nodes. The AlgImplTempl class
 * refers to an specific implementation derived from a generic algorithm.
 * 
 * First, ros::init is called providing the node name. Ctrl+C control is 
 * activated for sercure and safe exit. Finally, an AlgImplTempl object is
 * defined and launched to spin for endless loop execution.
 * 
 * \param argc integer with number of input parameters
 * \param argv array with all input strings
 * \param node_name name of the node
 */
template <class AlgImplTempl>
int main(int argc, char **argv, std::string node_name)
{
  ROS_DEBUG("IriBaseAlgorithm::%s Launched", node_name.c_str());

  // ROS initialization
  ros::init(argc, argv, node_name);

  // allow Ctrl+C management
  signal(SIGHUP, &AbstractAlgorithmNode::hupCalled);

  // define and launch generic algorithm implementation object
  AlgImplTempl algImpl;
  algImpl.spin();

  return 0;
}

}
#endif
