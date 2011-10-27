#include "tibi_dabo_head_driver_node.h"
#include "eventserver.h"
#include "XmlRpcValue.h"

TibiDaboHeadDriverNode::TibiDaboHeadDriverNode(ros::NodeHandle &nh) : iri_base_driver::IriBaseNodeDriver<TibiDaboHeadDriver>(nh),
  motion_sequence_aserver_(this->public_node_handle_, "motion_sequence"),
  lights_sequence_aserver_(this->public_node_handle_, "lights_sequence"),
  joint_motion_aserver_(this->public_node_handle_, "joint_motion")
{
  XmlRpc::XmlRpcValue files_param_list;
  XmlRpc::XmlRpcValue param_list_item;
  unsigned int i=0;

  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->joint_feedback_publisher_ = this->public_node_handle_.advertise<sensor_msgs::JointState>("joint_feedback", 100);
  
  // [init subscribers]
  this->facial_expression_subscriber_ = this->public_node_handle_.subscribe("facial_expression", 100, &TibiDaboHeadDriverNode::facial_expression_callback, this);

  // [init services]
  
  // [init clients]
  
  // [init action servers]
  motion_sequence_aserver_.registerStartCallback(boost::bind(&TibiDaboHeadDriverNode::motion_sequence_startCallback, this, _1));
  motion_sequence_aserver_.registerStopCallback(boost::bind(&TibiDaboHeadDriverNode::motion_sequence_stopCallback, this));
  motion_sequence_aserver_.registerIsFinishedCallback(boost::bind(&TibiDaboHeadDriverNode::motion_sequence_isFinishedCallback, this));
  motion_sequence_aserver_.registerHasSucceedCallback(boost::bind(&TibiDaboHeadDriverNode::motion_sequence_hasSucceedCallback, this));
  motion_sequence_aserver_.registerGetResultCallback(boost::bind(&TibiDaboHeadDriverNode::motion_sequence_getResultCallback, this, _1));
  motion_sequence_aserver_.registerGetFeedbackCallback(boost::bind(&TibiDaboHeadDriverNode::motion_sequence_getFeedbackCallback, this, _1));
  
  lights_sequence_aserver_.registerStartCallback(boost::bind(&TibiDaboHeadDriverNode::lights_sequence_startCallback, this, _1));
  lights_sequence_aserver_.registerStopCallback(boost::bind(&TibiDaboHeadDriverNode::lights_sequence_stopCallback, this));
  lights_sequence_aserver_.registerIsFinishedCallback(boost::bind(&TibiDaboHeadDriverNode::lights_sequence_isFinishedCallback, this));
  lights_sequence_aserver_.registerHasSucceedCallback(boost::bind(&TibiDaboHeadDriverNode::lights_sequence_hasSucceedCallback, this));
  lights_sequence_aserver_.registerGetResultCallback(boost::bind(&TibiDaboHeadDriverNode::lights_sequence_getResultCallback, this, _1));
  lights_sequence_aserver_.registerGetFeedbackCallback(boost::bind(&TibiDaboHeadDriverNode::lights_sequence_getFeedbackCallback, this, _1));

  joint_motion_aserver_.registerStartCallback(boost::bind(&TibiDaboHeadDriverNode::joint_motion_startCallback, this, _1));
  joint_motion_aserver_.registerStopCallback(boost::bind(&TibiDaboHeadDriverNode::joint_motion_stopCallback, this));
  joint_motion_aserver_.registerIsFinishedCallback(boost::bind(&TibiDaboHeadDriverNode::joint_motion_isFinishedCallback, this));
  joint_motion_aserver_.registerHasSucceedCallback(boost::bind(&TibiDaboHeadDriverNode::joint_motion_hasSucceedCallback, this));
  joint_motion_aserver_.registerGetResultCallback(boost::bind(&TibiDaboHeadDriverNode::joint_motion_getResultCallback, this, _1));
  joint_motion_aserver_.registerGetFeedbackCallback(boost::bind(&TibiDaboHeadDriverNode::joint_motion_getFeedbackCallback, this, _1));
  // [init action clients]
  // [init action clients]
  // publish parameters
  files_param_list.setSize(this->driver_.get_num_motion_seq_files());
  for(i=0;i<this->driver_.get_num_motion_seq_files();i++)
  {
    param_list_item=this->driver_.get_motion_seq_file(i);
    files_param_list[i]=param_list_item; 
  }
  this->public_node_handle_.setParam("motion_files",files_param_list);
  files_param_list.clear();
  files_param_list.setSize(this->driver_.get_num_config_files());
  for(i=0;i<this->driver_.get_num_config_files();i++)
  {
    param_list_item=this->driver_.get_config_file(i);
    files_param_list[i]=param_list_item; 
  }
  this->public_node_handle_.setParam("config_files",files_param_list);
  files_param_list.clear();
  files_param_list.setSize(this->driver_.get_num_lights_seq_files());
  for(i=0;i<this->driver_.get_num_lights_seq_files();i++)
  {
    param_list_item=this->driver_.get_lights_seq_file(i);
    files_param_list[i]=param_list_item; 
  }
  this->public_node_handle_.setParam("lights_files",files_param_list);
  files_param_list.clear();
}

void TibiDaboHeadDriverNode::mainNodeThread(void)
{
  std::vector<float> position(3);
  std::vector<float> velocity(3);
  unsigned int i=0;

  //lock access to driver if necessary
  this->driver_.lock();

  // update the loop rate
  this->loop_rate_ = this->driver_.get_feedback_rate();//in [Hz]

  // [fill msg Header if necessary]
  this->JointState_msg_.header.stamp = ros::Time::now();
  //<publisher_name>.header.frame_id = "<publisher_topic_name>";

  // [fill msg structures]
  this->driver_.get_position(position);
  this->driver_.get_velocity(velocity);
  this->JointState_msg_.position.clear();
  this->JointState_msg_.velocity.clear();
  this->JointState_msg_.name.clear();
  this->JointState_msg_.name.push_back("pan");
  this->JointState_msg_.name.push_back("tilt");
  this->JointState_msg_.name.push_back("side");
  for(i=0;i<position.size();i++)
  {
    this->JointState_msg_.position.push_back(position[i]*3.14159/180.0);
    this->JointState_msg_.velocity.push_back(velocity[i]*3.14159/180.0);
  }
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->joint_feedback_publisher_.publish(this->JointState_msg_);

  //unlock access to driver if previously blocked
  this->driver_.unlock();
}

void TibiDaboHeadDriverNode::check_motion_sequence_status(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  CEventServer *event_server=CEventServer::instance();

  if(driver_.isRunning())
  {
    if(this->driver_.get_motion_sequence_complete_event_id()!="")
    {
      if(event_server->event_is_set(this->driver_.get_motion_sequence_complete_event_id()))
      {
        if(event_server->event_is_set(this->driver_.get_motion_sequence_error_event_id()))
        {
          stat.summary(1,"Sequence has finished with errors");
          stat.add("Reported error",this->driver_.get_motion_sequence_error_message());
        }
        else
        {
          stat.summary(0,"Sequence has finished ok");
        }
      }
      else
        stat.summary(0,"Sequence is in progress");
    }
    else
      stat.summary(0,"No sequence has been executed");
  }
}

void TibiDaboHeadDriverNode::check_lights_sequence_status(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  CEventServer *event_server=CEventServer::instance();

  if(driver_.isRunning())
  {
    if(this->driver_.get_lights_sequence_complete_event_id()!="")
    {
      if(event_server->event_is_set(this->driver_.get_lights_sequence_complete_event_id()))
        stat.summary(0,"Sequence has finished ok");
      else
        stat.summary(0,"Sequence is in progress");
    }
    else
      stat.summary(0,"No sequence has been executed");
  }
}


/*  [subscriber callbacks] */
void TibiDaboHeadDriverNode::facial_expression_callback(const std_msgs::String::ConstPtr& msg) 
{
  tibi_dabo_msgs::sequenceResult result;
  std::string expression;
 
  ROS_DEBUG("TibiDaboHeadDriverNode::facial_expression_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->driver_.lock(); 
  //this->facial_expression_mutex_.enter(); 

  this->driver_.lock();
  if( driver_.isRunning() )
  {
    if(this->lights_sequence_aserver_.isActive())
    {
      result.successful.push_back(false);
      result.observations.push_back(std::string("A face expression has been set"));
      this->lights_sequence_aserver_.setAborted(result);
      this->driver_.stop_lights_sequence();
    }
    expression=msg->data;
    this->driver_.set_brightness(80.0);
    this->driver_.set_face_expression(expression);
  }
  this->driver_.unlock();
  //unlock previously blocked shared variables 
  //this->driver_.unlock(); 
  //this->facial_expression_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */
void TibiDaboHeadDriverNode::motion_sequence_startCallback(const tibi_dabo_msgs::sequenceGoalConstPtr& goal)
{
  std::string goal_file;
  tibi_dabo_msgs::sequenceResult result;

  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    if(this->joint_motion_aserver_.isActive())
    {
      result.successful.push_back(false);
      result.observations.push_back(std::string("A joint motion is in progress"));
      this->motion_sequence_aserver_.setAborted(result);
    }
    else
    {
      try{
        // add the full path to the incomming filename
        goal_file=motion_path + goal->sequence_file[0];
        std::cout << goal_file << std::endl;
        this->driver_.start_motion_sequence(goal_file);
      }catch(CException &e){
        result.successful.push_back(false);
        result.observations.push_back(e.what());
        this->motion_sequence_aserver_.setAborted(result);
      }
    }
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}

void TibiDaboHeadDriverNode::motion_sequence_stopCallback(void)
{
  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    try{
      this->driver_.stop_motion_sequence();
    }catch(CException &e){
      this->driver_.unlock();
      throw;
    }
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}

bool TibiDaboHeadDriverNode::motion_sequence_isFinishedCallback(void)
{
  CEventServer *event_server=CEventServer::instance();
  bool finished=false;

  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    finished=event_server->event_is_set(this->driver_.get_motion_sequence_complete_event_id());
  }

  //unlock access to driver if necessary
  this->driver_.unlock();

  return finished;
}

bool TibiDaboHeadDriverNode::motion_sequence_hasSucceedCallback(void)
{
  CEventServer *event_server=CEventServer::instance();
  bool success=false;

  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    success=!event_server->event_is_set(this->driver_.get_motion_sequence_error_event_id());
  }

  //unlock access to driver if necessary
  this->driver_.unlock();

  return success;
}

void TibiDaboHeadDriverNode::motion_sequence_getResultCallback(tibi_dabo_msgs::sequenceResultPtr& result)
{
  CEventServer *event_server=CEventServer::instance();

  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    result->successful.clear();
    result->observations.clear();
    result->successful.push_back(!event_server->event_is_set(this->driver_.get_motion_sequence_error_event_id()));
    result->observations.push_back(this->driver_.get_motion_sequence_error_message());
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}

void TibiDaboHeadDriverNode::motion_sequence_getFeedbackCallback(tibi_dabo_msgs::sequenceFeedbackPtr& feedback)
{
  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    try{
      feedback->percentage.clear();
      feedback->percentage.push_back(this->driver_.get_motion_sequence_completed_percentage());
    }catch(CException &e){
      this->driver_.unlock();
      throw;
    }
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}

void TibiDaboHeadDriverNode::lights_sequence_startCallback(const tibi_dabo_msgs::sequenceGoalConstPtr& goal)
{
  std::string goal_file;

  //lock access to driver if necessary
  this->driver_.lock();
  if( driver_.isRunning() )
  {
    try{
      // add the full path to the incomming filename
      goal_file=lights_path + goal->sequence_file[0];
      this->driver_.start_lights_sequence(goal_file);
    }catch(CException &e){
      this->driver_.unlock();
      throw;
    }
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}

void TibiDaboHeadDriverNode::lights_sequence_stopCallback(void)
{
  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    try{
      this->driver_.stop_lights_sequence();
    }catch(CException &e){
      this->driver_.unlock();
      throw;
    }
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}

bool TibiDaboHeadDriverNode::lights_sequence_isFinishedCallback(void)
{
  CEventServer *event_server=CEventServer::instance();
  bool finished=false;

  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    finished=event_server->event_is_set(this->driver_.get_lights_sequence_complete_event_id());
  }

  //unlock access to driver if necessary
  this->driver_.unlock();

  return finished;
}

bool TibiDaboHeadDriverNode::lights_sequence_hasSucceedCallback(void)
{
  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
  }

  //unlock access to driver if necessary
  this->driver_.unlock();

  return true;
}

void TibiDaboHeadDriverNode::lights_sequence_getResultCallback(tibi_dabo_msgs::sequenceResultPtr& result)
{
  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    result->successful.clear();
    result->successful.push_back(true);
    result->observations.clear();
    result->observations.push_back(std::string("Face expressions sequence executed properly"));
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}

void TibiDaboHeadDriverNode::lights_sequence_getFeedbackCallback(tibi_dabo_msgs::sequenceFeedbackPtr& feedback)
{
  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    try{
      feedback->percentage.clear();
      feedback->percentage.push_back(this->driver_.get_lights_sequence_completed_percentage());
    }catch(CException &e){
      this->driver_.unlock();
      throw;
    }
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}

void TibiDaboHeadDriverNode::joint_motion_startCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  control_msgs::FollowJointTrajectoryResult result;
  std::vector<TMotionStep> seq;
  unsigned int i=0,j=0;
  TMotionStep step;

  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    if(this->motion_sequence_aserver_.isActive())
      this->joint_motion_aserver_.setAborted(result,std::string("A motion sequence has been started"));
    else
    {
      try{
        // generate and execute the motion sequence
        for(i=0;i<goal->trajectory.points.size();i++)
        {
          step.position.clear();
          step.velocity.clear();
          for(j=0;j<goal->trajectory.points[i].positions.size();j++)
          {
            step.position.push_back(goal->trajectory.points[i].positions[j]);
            step.velocity.push_back(goal->trajectory.points[i].velocities[j]);
          }
          step.delay=goal->trajectory.points[i].time_from_start.toSec()*1000;
          seq.push_back(step);
        }
        this->driver_.start_motion_sequence(seq);
      }catch(CException &e){
        this->joint_motion_aserver_.setAborted(result);
      }
    }
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}

void TibiDaboHeadDriverNode::joint_motion_stopCallback(void)
{
  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    try{
      this->driver_.stop();
    }catch(CException &e){
      this->driver_.unlock();
      throw;
    }
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}

bool TibiDaboHeadDriverNode::joint_motion_isFinishedCallback(void)
{
  CEventServer *event_server=CEventServer::instance();
  bool finished=false;

  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    finished=event_server->event_is_set(this->driver_.get_motion_sequence_complete_event_id());
  }

  //unlock access to driver if necessary
  this->driver_.unlock();

  return finished;
}

bool TibiDaboHeadDriverNode::joint_motion_hasSucceedCallback(void)
{
  CEventServer *event_server=CEventServer::instance();

  //lock access to driver if necessary
  this->driver_.lock();
  bool success=false;

  if( driver_.isRunning() )
  {
    success=!event_server->event_is_set(this->driver_.get_motion_sequence_error_event_id());
  }

  //unlock access to driver if necessary
  this->driver_.unlock();

  return success;
}

void TibiDaboHeadDriverNode::joint_motion_getResultCallback(control_msgs::FollowJointTrajectoryResultPtr& result)
{
  CEventServer *event_server=CEventServer::instance();

  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    if(event_server->event_is_set(this->driver_.get_motion_sequence_error_event_id()))
      result->error_code=-1;
    else
      result->error_code=0;
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}

void TibiDaboHeadDriverNode::joint_motion_getFeedbackCallback(control_msgs::FollowJointTrajectoryFeedbackPtr& feedback)
{
  std::vector<float> position,velocity;
  unsigned int i=0;

  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    try{
      this->driver_.get_position(position);
      this->driver_.get_velocity(velocity); 
      for(i=0;i<position.size();i++)
      {
        feedback->actual.positions.push_back(position[i]);
        feedback->actual.velocities.push_back(velocity[i]);
      }
    }catch(CException &e){
      this->driver_.unlock();
      throw;
    }
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}
/*  [action requests] */

void TibiDaboHeadDriverNode::postNodeOpenHook(void)
{
  this->motion_sequence_aserver_.start();
  ROS_INFO("TibiDaboHeadDriverNode:: Motion sequence server started!");
  this->lights_sequence_aserver_.start();
  ROS_INFO("TibiDaboHeadDriverNode:: Lights sequence server started!");
  this->joint_motion_aserver_.start();
  ROS_INFO("TibiDaboHeadDriverNode:: Joint motion server started!");
}

void TibiDaboHeadDriverNode::preNodeCloseHook(void)
{
  if(this->motion_sequence_aserver_.isStarted())
  {
    this->motion_sequence_aserver_.shutdown(); 
    ROS_INFO("TibiDaboHeadDriverNode:: Motion sequence server stopped!");
  }
  if(this->lights_sequence_aserver_.isStarted())
  {
    this->lights_sequence_aserver_.shutdown(); 
    ROS_INFO("TibiDaboHeadDriverNode:: Lights sequence server stopped!");
  }
  if(this->joint_motion_aserver_.isStarted())
  {
    this->joint_motion_aserver_.shutdown(); 
    ROS_INFO("TibiDaboHeadDriverNode:: Joint motion server stopped!");
  }
//  ros::shutdown();
}

void TibiDaboHeadDriverNode::addNodeDiagnostics(void)
{
  diagnostic_.add("Motion sequence status", this, &TibiDaboHeadDriverNode::check_motion_sequence_status);
  diagnostic_.add("Lights sequence status", this, &TibiDaboHeadDriverNode::check_lights_sequence_status);
}

void TibiDaboHeadDriverNode::addNodeOpenedTests(void)
{
}

void TibiDaboHeadDriverNode::addNodeStoppedTests(void)
{
}

void TibiDaboHeadDriverNode::addNodeRunningTests(void)
{
}

void TibiDaboHeadDriverNode::reconfigureNodeHook(int level)
{
}

TibiDaboHeadDriverNode::~TibiDaboHeadDriverNode()
{
  this->driver_.lock();
  // [free dynamic memory]
  if(this->motion_sequence_aserver_.isActive())
  {
    this->motion_sequence_aserver_.setAborted();
    this->driver_.stop_motion_sequence();
  } 
  if(this->lights_sequence_aserver_.isActive())
  {
    this->lights_sequence_aserver_.setAborted();
    this->driver_.stop_lights_sequence();
  } 
  this->driver_.unlock();
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<TibiDaboHeadDriverNode>(argc, argv, "tibi_dabo_head_driver_node");
}
