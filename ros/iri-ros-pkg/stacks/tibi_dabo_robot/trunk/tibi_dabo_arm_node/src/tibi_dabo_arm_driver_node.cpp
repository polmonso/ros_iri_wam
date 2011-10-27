#include "tibi_dabo_arm_driver_node.h"
#include "eventserver.h"
#include "XmlRpcValue.h"

TibiDaboArmDriverNode::TibiDaboArmDriverNode(ros::NodeHandle &nh) : iri_base_driver::IriBaseNodeDriver<TibiDaboArmDriver>(nh),
  motion_sequence_aserver_(this->public_node_handle_, "motion_sequence"),
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
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  motion_sequence_aserver_.registerStartCallback(boost::bind(&TibiDaboArmDriverNode::motion_sequence_startCallback, this, _1));
  motion_sequence_aserver_.registerStopCallback(boost::bind(&TibiDaboArmDriverNode::motion_sequence_stopCallback, this));
  motion_sequence_aserver_.registerIsFinishedCallback(boost::bind(&TibiDaboArmDriverNode::motion_sequence_isFinishedCallback, this));
  motion_sequence_aserver_.registerHasSucceedCallback(boost::bind(&TibiDaboArmDriverNode::motion_sequence_hasSucceedCallback, this));
  motion_sequence_aserver_.registerGetResultCallback(boost::bind(&TibiDaboArmDriverNode::motion_sequence_getResultCallback, this, _1));
  motion_sequence_aserver_.registerGetFeedbackCallback(boost::bind(&TibiDaboArmDriverNode::motion_sequence_getFeedbackCallback, this, _1));

  joint_motion_aserver_.registerStartCallback(boost::bind(&TibiDaboArmDriverNode::joint_motion_startCallback, this, _1));
  joint_motion_aserver_.registerStopCallback(boost::bind(&TibiDaboArmDriverNode::joint_motion_stopCallback, this));
  joint_motion_aserver_.registerIsFinishedCallback(boost::bind(&TibiDaboArmDriverNode::joint_motion_isFinishedCallback, this));
  joint_motion_aserver_.registerHasSucceedCallback(boost::bind(&TibiDaboArmDriverNode::joint_motion_hasSucceedCallback, this));
  joint_motion_aserver_.registerGetResultCallback(boost::bind(&TibiDaboArmDriverNode::joint_motion_getResultCallback, this, _1));
  joint_motion_aserver_.registerGetFeedbackCallback(boost::bind(&TibiDaboArmDriverNode::joint_motion_getFeedbackCallback, this, _1));
  
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
}

void TibiDaboArmDriverNode::mainNodeThread(void)
{
  std::vector<float> position;
  std::vector<float> velocity;
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
  for(i=0;i<position.size();i++)
  {
    this->JointState_msg_.position.push_back(position[i]);
    this->JointState_msg_.velocity.push_back(velocity[i]);
  }
  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->joint_feedback_publisher_.publish(this->JointState_msg_);

  //unlock access to driver if previously blocked
  this->driver_.unlock();
}

void TibiDaboArmDriverNode::check_motion_sequence_status(diagnostic_updater::DiagnosticStatusWrapper &stat)
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

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */
void TibiDaboArmDriverNode::motion_sequence_startCallback(const tibi_dabo_msgs::sequenceGoalConstPtr& goal)
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

void TibiDaboArmDriverNode::motion_sequence_stopCallback(void)
{
  //lock access to driver if necessary

  if( driver_.isRunning() )
  {
    try{
      this->driver_.lock();
      this->driver_.stop_motion_sequence();
      this->driver_.unlock();
    }catch(CException &e){
      this->driver_.unlock();
      throw;
    }
  }

  //unlock access to driver if necessary
}

bool TibiDaboArmDriverNode::motion_sequence_isFinishedCallback(void)
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

bool TibiDaboArmDriverNode::motion_sequence_hasSucceedCallback(void)
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

void TibiDaboArmDriverNode::motion_sequence_getResultCallback(tibi_dabo_msgs::sequenceResultPtr& result)
{
  CEventServer *event_server=CEventServer::instance();

  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    result->successful.clear();
    result->successful.push_back(!event_server->event_is_set(this->driver_.get_motion_sequence_error_event_id()));
    result->observations.clear();
    result->observations.push_back(this->driver_.get_motion_sequence_error_message());
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}

void TibiDaboArmDriverNode::motion_sequence_getFeedbackCallback(tibi_dabo_msgs::sequenceFeedbackPtr& feedback)
{
  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    try{
      feedback->percentage.push_back(this->driver_.get_motion_sequence_completed_percentage());
    }catch(CException &e){
      this->driver_.unlock();
      throw;
    }
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}

void TibiDaboArmDriverNode::joint_motion_startCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
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
          step.delay=0;
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

void TibiDaboArmDriverNode::joint_motion_stopCallback(void)
{
  //lock access to driver if necessary
  if( driver_.isRunning() )
  {
    try{
      this->driver_.lock();
      this->driver_.stop();
      this->driver_.unlock();
    }catch(CException &e){
      this->driver_.unlock();
      throw;
    }
  }

  //unlock access to driver if necessary
}

bool TibiDaboArmDriverNode::joint_motion_isFinishedCallback(void)
{
  CEventServer *event_server=CEventServer::instance();
  bool finished=false;

  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
    finished=event_server->event_is_set(this->driver_.get_position_reached_event_id());
  }

  //unlock access to driver if necessary
  this->driver_.unlock();

  return finished;
}

bool TibiDaboArmDriverNode::joint_motion_hasSucceedCallback(void)
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

void TibiDaboArmDriverNode::joint_motion_getResultCallback(control_msgs::FollowJointTrajectoryResultPtr& result)
{
  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
  {
  }

  //unlock access to driver if necessary
  this->driver_.unlock();
}

void TibiDaboArmDriverNode::joint_motion_getFeedbackCallback(control_msgs::FollowJointTrajectoryFeedbackPtr& feedback)
{
  std::vector<float> position,velocity;

  //lock access to driver if necessary

  if( driver_.isRunning() )
  {
    try{
      this->driver_.lock();
      this->driver_.get_position(position);
      this->driver_.get_velocity(velocity);
      this->driver_.unlock();
    }catch(CException &e){
      this->driver_.unlock();
      throw;
    }
  }

  //unlock access to driver if necessary
}

/*  [action requests] */

void TibiDaboArmDriverNode::postNodeOpenHook(void)
{
  motion_sequence_aserver_.start();
  ROS_INFO("TibiDaboArmDriverNode:: Motion sequence server started!");
  joint_motion_aserver_.start();
  ROS_INFO("TibiDaboArmDriverNode:: Joint motion server started!");
}

void TibiDaboArmDriverNode::preNodeCloseHook(void)
{
  if(this->motion_sequence_aserver_.isStarted())
  {
    this->motion_sequence_aserver_.shutdown();
    ROS_INFO("TibiDaboArmDriverNode:: Motion sequence server stopped!");
  }
  if(this->joint_motion_aserver_.isStarted())
  {
    this->joint_motion_aserver_.shutdown();
    ROS_INFO("TibiDaboArmDriverNode:: Joint motion server stopped!");
  }
}

void TibiDaboArmDriverNode::addNodeDiagnostics(void)
{
  diagnostic_.add("Motion sequence status", this, &TibiDaboArmDriverNode::check_motion_sequence_status);
}

void TibiDaboArmDriverNode::addNodeOpenedTests(void)
{
}

void TibiDaboArmDriverNode::addNodeStoppedTests(void)
{
}

void TibiDaboArmDriverNode::addNodeRunningTests(void)
{
}

void TibiDaboArmDriverNode::reconfigureNodeHook(int level)
{
}

TibiDaboArmDriverNode::~TibiDaboArmDriverNode()
{
  // [free dynamic memory]
  this->driver_.lock();
  // [free dynamic memory]
  if(this->motion_sequence_aserver_.isActive())
  {
    this->motion_sequence_aserver_.setAborted();
    this->driver_.stop_motion_sequence();
  }
  this->driver_.unlock();

}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<TibiDaboArmDriverNode>(argc, argv, "tibi_dabo_arm_driver_node");
}
