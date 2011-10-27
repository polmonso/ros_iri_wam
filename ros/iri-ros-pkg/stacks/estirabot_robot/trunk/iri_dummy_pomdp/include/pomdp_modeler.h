#ifndef _pomdp_modeler_h_
#define _pomdp_modeler_h_

#include <iostream>
#include <sstream>

#include "ros/this_node.h"
#include "ros/names.h"
#include "ros/node_handle.h"
#include "ros/rate.h"

#include "mutex.h"
#include "Modeler.h"

// [publisher subscriber headers]

// [service client headers]
#include "std_srvs/Empty.h"
#include "std_msgs/Int32.h"
#include "iri_wam_common_msgs/wamaction.h"
#include "iri_wam_common_msgs/obs_request.h"
#include "iri_wam_common_msgs/modeling.h"

// [action server client headers]

#define NUMMAX 5

class PomdpModeler
{
  private:
    bool execution_flag;

    ros::NodeHandle node_handle_;
    
    // [publisher attributes]
    ros::Publisher focused_obj_label_publisher;
    std_msgs::Int32 progress_msg;

    // [subscriber attributes]
        
    // [service attributes]
    ros::ServiceServer execution_flow_control_server;
    bool execution_flow_controlCallback(iri_wam_common_msgs::modeling::Request &req, iri_wam_common_msgs::modeling::Response &res);
    
            
    // [client attributes]
    ros::ServiceClient obs_client;
    iri_wam_common_msgs::obs_request obs_request_srv;
    ros::ServiceClient save_pcl_client;
    std_srvs::Empty save_pcl_srv;
    ros::ServiceClient wam_action_client;
    iri_wam_common_msgs::wamaction wam_actions_client_srv;
            
    // [action server attributes]
            
    // [action client attributes]

    //put inside modeler
//    std::vector<std::vector<double> > rewardMatrix; // double rewardMatrix[NUMACTIONS][NUMSTATES];
//    array3d transform; // double transform[NUMACTIONS][NUMSTATES][NUMSTATES];
//    array3d observations; // double observations[NUMACTIONS][NUMSTATES][NUMOBSERVATIONS];
    double obs_base_matrix[5][5];

    Modeler *modeler;
    int experiments_per_action;
//    void copyStateFamilies();
    int numactions;
    int modelactions;
    int numstates;
    int numobservations;

    int counter;
    int state_final;
    int real_num_objects;
    void printObsBaseMatrix();

  public:
   /**      
    * \brief constructor
    *
    * This constructor mainly creates and initializes the WamActions topics
    * through the given NodeHandle object. CIriNode attributes may be also
    * modified to suit node specifications.
    *
    * All kind of ROS topics (publishers, subscribers, servers or clients) can 
    * be easyly generated with the scripts in the iri_ros_scripts package. Refer
    * to ROS and IRI Wiki pages for more details:
    *
    * http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber(c++)
    * http://www.ros.org/wiki/ROS/Tutorials/WritingServiceClient(c++)
    * http://wikiri.upc.es/index.php/Robotics_Lab
    *
    * \param nh a reference to the node handle object to manage all ROS topics.
    */
    PomdpModeler();

   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the driver is on running state. Loop frequency 
    * can be tuned my modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainLoop(void);

};
#endif

