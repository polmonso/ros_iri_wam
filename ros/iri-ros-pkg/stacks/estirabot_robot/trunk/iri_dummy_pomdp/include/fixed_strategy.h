#ifndef _fixed_strategy_h_
#define _fixed_strategy_h_

#include "ros/this_node.h"
#include "ros/names.h"
#include "ros/node_handle.h"
#include "ros/rate.h"

#include "mutex.h"

// [publisher subscriber headers]

// [service client headers]
#include "std_srvs/Empty.h"
#include "std_msgs/Int32.h"
#include "iri_wam_common_msgs/wamaction.h"
#include "iri_wam_common_msgs/obs_request.h"

// [action server client headers]

#define NUMPOSES 3

enum {
    //remember to change the other .h enums (dummy_pomdp)!
    FOLDLOW,
    FOLDHIGH,
    SPREAD,
    FLIP,
    SHAKE,
    CHANGEPILE,
    MOVEAWAY,
    DUMMY,
    TAKE
};

enum {
    STRAIGHT,
    ISOMETRIC,
    PEG
};

enum {
    ANY,
    A,
    B
};


class FixedStrategy
{
  private:
    CMutex execution_mutex;
    bool execution_flag;

    ros::NodeHandle node_handle_;
    
    // [publisher attributes]
    ros::Publisher focused_obj_label_publisher;
    std_msgs::Int32 ObjLabel_msg;

    // [subscriber attributes]
        
    // [service attributes]
    ros::ServiceServer execution_flow_control_server;
    bool execution_flow_controlCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
            
    // [client attributes]
    ros::ServiceClient obs_client;
    iri_wam_common_msgs::obs_request obs_request_srv;
    ros::ServiceClient wam_action_client;
    iri_wam_common_msgs::wamaction wam_actions_client_srv;
            
    // [action server attributes]
            
    // [action client attributes]
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
    FixedStrategy();

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

