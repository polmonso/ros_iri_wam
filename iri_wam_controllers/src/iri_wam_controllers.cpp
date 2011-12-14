#include "iri_wam_controllers.h"
#include <pluginlib/class_list_macros.h>
using namespace XmlRpc;


PLUGINLIB_DECLARE_CLASS(iri_wam_controller,WAMController,controller::WAMController, pr2_controller_interface::Controller)
namespace controller {

WAMController::WAMController():
state_(FREE)
{
}
/// Controller initialization in non-realtime
bool WAMController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  /* std::string ip_server;
  int port_server;
  int rate_refresh;
  if(state_ == FREE)
  {
  if (!n.getParam("ip_server", ip_server))
  {
    ROS_ERROR("No server ip  given in namespace: '%s')",n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("port_server", port_server))
  {
    ROS_ERROR("No server port  given in namespace: '%s')",n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("rate_refresh", rate_refresh))
  {
    ROS_ERROR("No server port  given in namespace: '%s')",n.getNamespace().c_str());
    return false;
  }
  state_=OPENED;
  }*/
  node_=n;
  XmlRpc::XmlRpcValue joint_names;
  if (!node_.getParam("joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)",node_.getNamespace().c_str());
    return false;
  }
  for (int i = 0; i < joint_names.size(); ++i)
  {
    XmlRpcValue &name_value = joint_names[i];
    if (name_value.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)", n.getNamespace().c_str());
      return false;
    }

    pr2_mechanism_model::JointState *j = robot->getJointState((std::string)name_value);
    if (!j) {
      ROS_ERROR("Joint not found: %s. (namespace: %s)",((std::string)name_value).c_str(), n.getNamespace().c_str());
      return false;
    }
    joints_names_.push_back((std::string)name_value);
  }  
  
  
  state_publisher=n.advertise<sensor_msgs::JointState>("joint_states", 5);	
  

  
  
  /*
  else
  {
	  //state_=WAITING;
	  ROS_WARN("one controllers is running ");
  }
 this->wam = new CWamDriver(this->wamserver_ip, this->server_port, this->state_refresh_rate);
  wam->open();
  this->state_ = OPENED;
  ROS_INFO("Wam opened, press shift+idle and enter.");
  getchar();
  wam->create(); 
  ROS_INFO("Wam created, press shift+activate and press enter.");
  getchar(); 
  wam->activate();*/
  return true;
}


/// Controller startup in realtime
void WAMController::starting()
{
    action_server_.reset(new JTAS(node_, "joint_trajectory_action", boost::bind(&WAMController::goalCB, this, _1), boost::bind(&WAMController::cancelCB, this, _1)));
    ros::service::waitForService("joints_move");	
    client=node_.serviceClient<iri_wam_common_msgs::joints_move> ("joints_move");	
  /*this->wam = new CWamDriver(this->wamserver_ip, this->server_port, this->state_refresh_rate);
  wam->open();
  state_=OPENED;
  ROS_INFO("Wam opened, press shift+idle and enter.");
  getchar();
  wam->create(); 
  ROS_INFO("Wam created, press shift+activate and press enter.");
  getchar(); 
  wam->activate();
  state_=WAITING;*/
}


/// Controller update loop in realtime
void WAMController::update()
{
  /*double desired_pos = init_pos_ + 15 * sin(ros::Time::now().toSec());
  double current_pos = joint_state_->position_;
  joint_state_->commanded_effort_ = -10 * (current_pos - desired_pos);*/
}


/// Controller stopping in realtime
void WAMController::stopping()
{
}
void WAMController::cancelCB(GoalHandle gh)
{
  /*boost::shared_ptr<RTGoalHandle> current_active_goal(rt_active_goal_);
  if (current_active_goal && current_active_goal->gh_ == gh)
  {
    rt_active_goal_.reset();

    trajectory_msgs::JointTrajectory::Ptr empty(new trajectory_msgs::JointTrajectory);
    empty->joint_names.resize(joints_.size());
    for (size_t j = 0; j < joints_.size(); ++j)
      empty->joint_names[j] = joints_[j]->joint_->name;
    commandTrajectory(empty);

    // Marks the current goal as canceled.
    current_active_goal->gh_.setCanceled();
  }*/
}
void WAMController::goalCB(GoalHandle gh)
{
  /*std::vector<std::string> joint_names(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
    joint_names[j] = joints_[j]->joint_->name;

  // Ensures that the joints in the goal match the joints we are commanding.
  if (!setsEqual(joint_names, gh.getGoal()->trajectory.joint_names))
  {
    ROS_ERROR("Joints on incoming goal don't match our joints");
    gh.setRejected();
    return;
  }

  preemptActiveGoal();

  gh.setAccepted();
  boost::shared_ptr<RTGoalHandle> rt_gh(new RTGoalHandle(gh));

  // Sends the trajectory along to the controller
  goal_handle_timer_ = node_.createTimer(ros::Duration(0.01), &RTGoalHandle::runNonRT, rt_gh);
  commandTrajectory(share_member(gh.getGoal(), gh.getGoal()->trajectory), rt_gh);
  rt_active_goal_ = rt_gh;
  goal_handle_timer_.start();*/
  if( gh.getGoal()->trajectory.points.size() == 0){
	  ROS_FATAL("No Hay Puntos");
	  return;
  }
  gh.setAccepted();
  
  
  iri_wam_common_msgs::joints_move::Request request;
  iri_wam_common_msgs::joints_move::Response response;
  request.joints=gh.getGoal()->trajectory.points[0].positions;
    if(client.call(request,response))
  {
	if(response.success)
	{
		ROS_INFO_STREAM("Operation succeful for "<<response.success);
	}
	else
	{
		ROS_INFO_STREAM("Operation Failure for "<<response.success);
	}
  }
  else
  {
	  ROS_FATAL("Service failure");
  }
}

} // namespace

