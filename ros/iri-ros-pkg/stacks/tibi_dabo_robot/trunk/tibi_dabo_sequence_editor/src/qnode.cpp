/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include "../include/tibi_dabo_sequence_editor/qnode.hpp"
#include "XmlRpcValue.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tibi_dabo_sequence_editor {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) 
{
  std::string config_files_list,config_file;
  unsigned int pos=0,new_pos=0;

  ros::init(argc,argv,"tibi_dabo_sequence_editor");// initialize ROS
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n(ros::this_node::getName());
  // initialize the parameter info
  if(!n.getParam("node_name",this->node_name))   
    emit rosShutdown();
  if(n.hasParam("config_files"))
  {
    n.getParam("config_files", config_files_list);
    while((new_pos=config_files_list.find(",",pos))!=std::string::npos)
    {
      config_file=std::string(getenv("IRI_ROS_STACK_PATH"))+std::string("/tibi_dabo_robot/")+this->node_name+std::string("/")+config_files_list.substr(pos,new_pos-pos);
      this->config_files.push_back(config_file);
      pos=new_pos+1;
    }
    config_file=std::string(getenv("IRI_ROS_STACK_PATH"))+std::string("/tibi_dabo_robot/")+this->node_name+std::string("/")+config_files_list.substr(pos);
    this->config_files.push_back(config_file);
  }
  else
    emit rosShutdown();

  // Add your ros communications here.
  // [init subscribers]
  this->motion_feedback_subscriber_ = n.subscribe("joint_feedback", 100, &QNode::motion_feedback_callback, this);
  // init action client
  this->motion_.client=new SeqClient("motion",true);
  this->motion_.active=false;
  this->motion_.succeeded=false;
  this->motion_.loaded=false;
  this->motion_.percentage=0.0;

  // initialize the events
  this->event_server=CEventServer::instance();
  this->new_feedback_event_id="new_feedback_event_id";
  this->event_server->create_event(this->new_feedback_event_id);
  this->action_feedback_event_id="action_feedback_event_id";
  this->event_server->create_event(this->action_feedback_event_id);
 
  start();// start the thread
}

QNode::~QNode() 
{
  if(this->new_feedback_event_id!="")
  {
    this->event_server->delete_event(this->new_feedback_event_id);
    this->new_feedback_event_id="";
  }
  if(this->action_feedback_event_id!="")
  {
    this->event_server->delete_event(this->action_feedback_event_id);
    this->action_feedback_event_id="";
  }

  if(ros::isStarted()) 
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();// wait for the thread to end
}

void QNode::run() // main thread function
{
  ros::Rate loop_rate(100);// set the loop rate

  while ( ros::ok() ) 
  {
    if(!this->motion_.client->isServerConnected())
    {
    }
    // Add your ros communications here.
    ros::spinOnce();
    loop_rate.sleep();
  }
  emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::get_motion_feedback(std::vector<float> &position, std::vector<float> &velocity)
{
  unsigned int i=0;

  this->motion_feedback_mutex_.enter();
  for(i=0;i<this->position.size();i++)
  {
    position.push_back(this->position[i]);
    velocity.push_back(this->velocity[i]);
  }
  this->motion_feedback_mutex_.exit();
}

std::vector<std::string> QNode::get_config_files(void)
{
  return this->config_files;
}

bool QNode::is_connected(void)
{
  return this->motion_.client->isServerConnected();
}

std::string QNode::get_new_feedback_event_id(void)
{
  return this->new_feedback_event_id;
}

std::string QNode::get_action_feedback_event_id(void)
{
  return this->action_feedback_event_id;
}

void QNode::execute_sequence(std::vector<TMotionStep> &seq)
{
  control_msgs::FollowJointTrajectoryGoal goal;
  unsigned int i=0,j=0;
  
  // create the internal goal
  goal.trajectory.points.resize(seq.size());
  for(i=0;i<seq.size();i++)
  {
    for(j=0;j<seq[i].position.size();j++)
    {
      goal.trajectory.points[i].positions.push_back(seq[i].position[j]);
      goal.trajectory.points[i].velocities.push_back(seq[i].velocity[j]);
    }
    goal.trajectory.points[i].time_from_start=ros::Duration(seq[i].delay/1000.0);
  }
  // send the goal
  this->motionMakeActionRequest(goal);
}

void QNode::motion_feedback_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  unsigned int i=0;

  this->motion_feedback_mutex_.enter();

  this->position.clear();
  this->velocity.clear();
  for(i=0;i<msg->position.size();i++)
  {
    this->position.push_back(msg->position[i]);
    this->velocity.push_back(msg->velocity[i]);
  }
  if(!this->event_server->event_is_set(this->new_feedback_event_id))
    this->event_server->set_event(this->new_feedback_event_id);

  this->motion_feedback_mutex_.exit();
}

void QNode::motionMakeActionRequest(const control_msgs::FollowJointTrajectoryGoal & motion_goal)
{
  ROS_INFO("Starting New Request!");

  //wait for the action server to start 
  //will wait for infinite time 
  if(this->motion_.client->isServerConnected())
  {
    ROS_DEBUG("TibiDaboHriAlgNode::ttsMakeActionRequest: TTS Server is Available!");

    //send a goal to the action 
    this->motion_.client->sendGoal(motion_goal,
               boost::bind(&QNode::motionDone,     this, _1, _2),
               boost::bind(&QNode::motionActive,   this),
               boost::bind(&QNode::motionFeedback, this, _1));

    this->motion_.active=true;
    this->motion_.loaded=true;
  }
  else
  {
    throw std::string("No connection to the action server.");
  }
}

void QNode::motionDone(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO("Motion sequence done");
}

void QNode::motionActive(void)
{
  ROS_INFO("Motion sequence active");
}

void QNode::motionFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  ROS_INFO("Motion sequence feedback");
  if(!this->event_server->event_is_set(this->action_feedback_event_id))
    this->event_server->set_event(this->action_feedback_event_id);
}

}  // namespace tibi_dabo_sequence_editor
