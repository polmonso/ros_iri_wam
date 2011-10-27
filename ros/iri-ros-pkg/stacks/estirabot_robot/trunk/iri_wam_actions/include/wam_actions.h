#ifndef _wam_actions_h_
#define _wam_actions_h_

#include "ros/this_node.h"
#include "ros/names.h"
#include "ros/node_handle.h"
#include "ros/rate.h"

// [publisher subscriber headers]

// [service client headers]
#include "iri_wam_common_msgs/bhand_cmd.h"
#include "iri_wam_common_msgs/pose_move.h"
#include "iri_wam_common_msgs/wamaction.h"
#include "iri_wam_common_msgs/compute_obj_grasp_pose.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/tfMessage.h"
// [action server client headers]

#include "mutex.h"

#define NUMPOSES 3
#define AZONEMSG 1
#define BZONEMSG 2
#define ANYZONEMSG 0
#define AZONE 1000
#define BZONE 2000
#define ANYZONE 0

#define NOX 0.5
#define NOY 0
#define NOZ 0

enum {
    //remember to change the other .h enums (dummy_pomdp)!
    TAKEHIGH,
    TAKELOW,
    CHANGEPILE,
    FOLDLOW,
    SPREAD,
    FLIP,
    SHAKE,
    FOLDHIGH,
    MOVEAWAY,
    DUMMY
};

enum {
    STRAIGHT,
    ISOMETRIC,
    PEG
};

/**
 * \brief IRI ROS Specific Driver Class
 *
 * IMPORTANT NOTE: This code has been generated through a script from the 
 * iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
 * of the scripts. ROS topics can be easly add by using those scripts. Please
 * refer to the IRI Wiki page for more information:
 * http://wikiri.upc.es/index.php/Robotics_Lab
 *
 * This class inherits from the IRI Base class CIriNode<CIriDriver>, to provide 
 * an execution thread to the driver object. A complete framework with utilites
 * to test the node functionallity or to add diagnostics to specific situations
 * is also given. The inherit template design form allows complete access to 
 * any CIriDriver object implementation.
 *
 * As mentioned, tests in the different driver states can be performed through 
 * class methods such as addNodeOpenedTests() or addNodeRunningTests(). Tests
 * common to all nodes may be also executed in the pattern class CIriNode.
 * Similarly to the tests, diagnostics can easyly be added. See ROS Wiki for
 * more details:
 * http://www.ros.org/wiki/diagnostics/ (Tutorials: Creating a Diagnostic Analyzer)
 * http://www.ros.org/wiki/self_test/ (Example: Self Test)
 */
class WamActions
{
  private: 
    CMutex tf_broadcaster_mutex;
    ros::NodeHandle node_handle_;
    geometry_msgs::Pose sampleposes[NUMPOSES];
    tf::TransformBroadcaster tf_br;
    tf::TransformListener tf_grasp;

    // [publisher attributes]
    tf::Transform transform_grasping_point;

    // [subscriber attributes]

    // [service attributes]
    ros::ServiceServer wam_action_server;
    bool wam_actionCallback(iri_wam_common_msgs::wamaction::Request &req, iri_wam_common_msgs::wamaction::Response &res);

    // [client attributes]
    ros::ServiceClient obj_filter_client;
    iri_wam_common_msgs::compute_obj_grasp_pose obj_filter_srv;
    ros::ServiceClient barrett_hand_cmd_client;
    iri_wam_common_msgs::bhand_cmd barrett_hand_cmd_srv;
    ros::ServiceClient pose_move_client;
    iri_wam_common_msgs::pose_move pose_move_srv;

    // [action server attributes]

    // [action client attributes]
    bool check_box(double x, double y, double z);
    bool fit_box(double& x, double& y, double& z);
    bool go_to(double x, double y, double z); 
    bool go_to_kinect(); 

    int destination_counter;
    void get_destination(int destination_zone, double& x, double& y, double& z);


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
    WamActions();

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

    //new more general purpose actions
    bool requestGraspingPoint(int filterID, double &x, double &y, double &z);
    bool take3D(double x, double y, double z);
    bool drop();
    bool drop(double x, double y, double z);
    bool isograsp(); 
    bool peggrasp();

    //take related commands
    bool stretch();
    bool fold();
    bool fromAtoB();
    bool fromBtoA();
    bool flip();
    bool shake();


};
#endif
