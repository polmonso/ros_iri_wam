#include <boost/scoped_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>
#include <ros/node_handle.h>
#include <pr2_controller_interface/controller.h>
#include <sensor_msgs/JointState.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_box.h>
#include <actionlib/server/action_server.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include "iri_wam_common_msgs/joints_move.h"
//include wam_driver main library
//#include "CWamDriver.h"

typedef actionlib::ActionServer<pr2_controllers_msgs::JointTrajectoryAction> JTAS;
typedef JTAS::GoalHandle GoalHandle;


namespace controller{
 enum state{FREE,OPENED,RUNNIG,WAITING,CLOSE};

class WAMController: public pr2_controller_interface::Controller
{
private:
  //CWamDriver *wam;
  controller::state state_; 
  std::vector<std::string> joints_names_;
  // ROS STUFF
  ros::Publisher state_publisher;
    boost::scoped_ptr<JTAS> action_server_;
    ros::ServiceClient client;
    ros::NodeHandle node_;
    void cancelCB(GoalHandle gh);
    void goalCB(GoalHandle gh);
public:
  WAMController();
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();
  
};
} 

