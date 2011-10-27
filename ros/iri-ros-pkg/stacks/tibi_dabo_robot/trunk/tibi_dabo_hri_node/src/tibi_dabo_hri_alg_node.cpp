#include "tibi_dabo_hri_alg_node.h"

TibiDaboHriAlgNode::TibiDaboHriAlgNode(void) :
  seqs_aserver_(public_node_handle_, "hri")
{
  //init class attributes if necessary
  this->loop_rate_ = 100;//in [Hz]

  // [init publishers]
  facial_expression_publisher_ = public_node_handle_.advertise<std_msgs::String>("facial_expression", 100);
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  seqs_aserver_.registerStartCallback(boost::bind(&TibiDaboHriAlgNode::startCallback, this, _1));
  seqs_aserver_.registerStopCallback(boost::bind(&TibiDaboHriAlgNode::stopCallback, this));
  seqs_aserver_.registerIsFinishedCallback(boost::bind(&TibiDaboHriAlgNode::isFinishedCallback, this));
  seqs_aserver_.registerHasSucceedCallback(boost::bind(&TibiDaboHriAlgNode::hasSucceedCallback, this));
  seqs_aserver_.registerGetResultCallback(boost::bind(&TibiDaboHriAlgNode::getResultCallback, this, _1));
  seqs_aserver_.registerGetFeedbackCallback(boost::bind(&TibiDaboHriAlgNode::getFeedbackCallback, this, _1));

  // [init action clients]
  this->tts_.client=new TtsClient("tts", true),
  this->tts_.active=false;
  this->tts_.succeeded=false;
  this->tts_.loaded=false;
  this->tts_.percentage=0.0;
  this->leds_.client=new SeqClient("leds",true);
  this->leds_.active=false;
  this->leds_.succeeded=false;
  this->leds_.loaded=false;
  this->leds_.percentage=0.0;
  this->head_.client=new SeqClient("head",true);
  this->head_.active=false;
  this->head_.succeeded=false;
  this->head_.loaded=false;
  this->head_.percentage=0.0;
  this->left_arm_.client=new SeqClient("left_arm",true);
  this->left_arm_.active=false;
  this->left_arm_.succeeded=false;
  this->left_arm_.loaded=false;
  this->left_arm_.percentage=0.0;
  this->right_arm_.client=new SeqClient("right_arm",true);
  this->right_arm_.active=false;
  this->right_arm_.succeeded=false;
  this->right_arm_.loaded=false;
  this->right_arm_.percentage=0.0;
  
  srand(ros::Time::now().toSec());

  empty_leds_sequence_=false;
}

TibiDaboHriAlgNode::~TibiDaboHriAlgNode(void)
{
  alg_.lock();
  // stop any current actions
  if(this->seqs_aserver_.isActive())
  {
    this->cancelCurrentGoals();
    this->seqs_aserver_.setAborted();
  }
  delete this->tts_.client;
  delete this->leds_.client;
  delete this->head_.client;
  delete this->left_arm_.client;
  delete this->right_arm_.client;
  // [free dynamic memory]
  alg_.unlock();
}

void TibiDaboHriAlgNode::cancelCurrentGoals(void)
{
  ROS_DEBUG("TibiDaboHriAlgNode::cancelCurrentGoals!");

  if(this->tts_.client->isServerConnected() && this->tts_.loaded && this->tts_.active)
    this->tts_.client->cancelGoal();
  if(this->leds_.client->isServerConnected() && this->leds_.loaded && this->leds_.active)
    this->leds_.client->cancelGoal();
  if(this->head_.client->isServerConnected() && this->head_.loaded && this->head_.active)
    this->head_.client->cancelGoal();
  if(this->left_arm_.client->isServerConnected() && this->left_arm_.loaded && this->left_arm_.active)
    this->left_arm_.client->cancelGoal();
  if(this->right_arm_.client->isServerConnected() && this->right_arm_.loaded && this->right_arm_.active)
    this->right_arm_.client->cancelGoal();
}

void TibiDaboHriAlgNode::speak(void)
{
  static int count = 0;
  std_msgs::String msg;
 
  if(count==0)
  {
    count=rand() % 200 + 200;
    msg.data = "speak";
    facial_expression_publisher_.publish(msg);
  }
  else 
  {
    count-=10;
    if(count<0) count=0;
  }
}

void TibiDaboHriAlgNode::mainNodeThread(void)
{
  alg_.lock();
  
  // check availability of all sequence clients
  bool all_clients_connected = true;

  // update client actions state
  if(!this->tts_.client->isServerConnected())
    all_clients_connected=false;
  if(!this->leds_.client->isServerConnected())
    all_clients_connected=false;
  if(!this->head_.client->isServerConnected())
    all_clients_connected=false;
  if(!this->left_arm_.client->isServerConnected())
    all_clients_connected=false;
  if(!this->right_arm_.client->isServerConnected())
    all_clients_connected=false;
  
  // if all sequence clients and tts client are active
  if( all_clients_connected )
  {
    // allow incoming action requests
    if( !this->seqs_aserver_.isStarted() )
    {
      this->seqs_aserver_.start();
      ROS_INFO("TibiDaboHriAlgNode:: Server Started!"); 
    }
  }
  else  // some clients are not alive
  {
    // if action server is active 
    if( this->seqs_aserver_.isActive() )
    {
      //preempt clients' current goals
      this->cancelCurrentGoals();

      // disable incoming action requests
      this->seqs_aserver_.setAborted();
    }

    // if action server is started
    if( this->seqs_aserver_.isStarted() )
    {
      // shutdown until all clients are available again
      this->seqs_aserver_.shutdown();
      ROS_INFO("TibiDaboHriAlgNode:: Server Stop!"); 
    }
  }
  
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  if( this->tts_.active && empty_leds_sequence_ )
    this->speak();
  alg_.unlock();
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

///////////////////////////////////////////////////////////////////////////////
//   Sequence Action Server
///////////////////////////////////////////////////////////////////////////////
void TibiDaboHriAlgNode::startCallback(const tibi_dabo_msgs::sequenceGoalConstPtr& goal)
{
  //lock access to driver if necessary
  alg_.lock();
  
  ROS_INFO("TibiDaboHriAlgNode::startCallback:");

  iri_common_drivers_msgs::ttsGoal      tts_goal;
  tibi_dabo_msgs::sequenceGoal leds_goal;
  tibi_dabo_msgs::sequenceGoal head_goal;
  tibi_dabo_msgs::sequenceGoal left_arm_goal;
  tibi_dabo_msgs::sequenceGoal right_arm_goal;
  
  // load sequences
  tts_goal.msg = goal->sequence_file[TTS_];
  leds_goal.sequence_file.push_back( goal->sequence_file[LEDS_] );
  head_goal.sequence_file.push_back( goal->sequence_file[HEAD_] );
  left_arm_goal.sequence_file.push_back( goal->sequence_file[LEFT_ARM_] );
  right_arm_goal.sequence_file.push_back( goal->sequence_file[RIGHT_ARM_] );

  // make requests to all action servers
  // if goal sequence is not empty
  if(tts_goal.msg.size() != 0)
  {
    ttsMakeActionRequest(tts_goal);
    this->tts_.loaded=true;
  }

  if(leds_goal.sequence_file[0].size() != 0)
  {
    ledsMakeActionRequest(leds_goal);
    empty_leds_sequence_ = false;
    this->leds_.loaded=true;
  }
  else
    empty_leds_sequence_ = true;

  if(head_goal.sequence_file[0].size() != 0)
  {
    this->head_.loaded=true;
    headMakeActionRequest(head_goal);
  }
    
  if(left_arm_goal.sequence_file[0].size() != 0)
  {
    this->left_arm_.loaded=true;
    leftArmMakeActionRequest(left_arm_goal);
  }
    
  if(right_arm_goal.sequence_file[0].size() != 0)
  {
    this->right_arm_.loaded=true;
    rightArmMakeActionRequest(right_arm_goal);
  }
    
  //lock access to driver if necessary
  alg_.unlock();
}

void TibiDaboHriAlgNode::stopCallback(void)
{
  ROS_DEBUG("TibiDaboHriAlgNode::stopCallback");
  alg_.lock();
  this->cancelCurrentGoals();
  alg_.unlock();
}

bool TibiDaboHriAlgNode::isFinishedCallback(void)
{
  ROS_DEBUG("TibiDaboHriAlgNode::isFinishedCallback");

  bool finish = true;

  alg_.lock();

  if(this->tts_.loaded && this->tts_.active)
    finish=false;
  if(this->leds_.loaded && this->leds_.active)
    finish=false;
  if(this->head_.loaded && this->head_.active)
    finish=false;
  if(this->left_arm_.loaded && this->left_arm_.active)
    finish=false;
  if(this->right_arm_.loaded && this->right_arm_.active)
    finish=false;

  alg_.unlock();
//   if(finish)
//     ROS_INFO("TibiDaboHriAlgNode::isFinishedCallback - FINISHED!!!");
//   else
//     ROS_INFO("TibiDaboHriAlgNode::isFinishedCallback - NOT FINISHED YET!!!");
    

  return finish;
}

bool TibiDaboHriAlgNode::hasSucceedCallback(void)
{
  ROS_DEBUG("TibiDaboHriAlgNode::hasSucceedCallback");

  bool succeed = true;

  alg_.lock();

  if(this->tts_.loaded && !this->tts_.succeeded)
  {
    succeed=false;
    this->tts_.loaded=false;
  }
  if(this->leds_.loaded && !this->leds_.succeeded)
  {
    succeed=false;
    this->leds_.loaded=false;
  }
  if(this->head_.loaded && !this->head_.succeeded)
  {
    succeed=false;
    this->head_.loaded=false;
  }
  if(this->left_arm_.loaded && !this->left_arm_.succeeded)
  {
    succeed=false;
    this->left_arm_.loaded=false;
  }
  if(this->right_arm_.loaded && !this->right_arm_.succeeded)
  {
    succeed=false;
    this->right_arm_.loaded=false;
  }

  alg_.unlock();
  
  return succeed;
}

void TibiDaboHriAlgNode::getResultCallback(tibi_dabo_msgs::sequenceResultPtr& result)
{
  //lock access to driver if necessary
  alg_.lock();

  result->successful.push_back(this->tts_.succeeded);
  result->successful.push_back(this->leds_.succeeded);
  result->successful.push_back(this->head_.succeeded);
  result->successful.push_back(this->left_arm_.succeeded);
  result->successful.push_back(this->right_arm_.succeeded);
  
  //lock access to driver if necessary
  alg_.unlock();
}

void TibiDaboHriAlgNode::getFeedbackCallback(tibi_dabo_msgs::sequenceFeedbackPtr& feedback)
{
  //lock access to driver if necessary
  alg_.lock();
  
  feedback->percentage.push_back(this->tts_.percentage);
  feedback->percentage.push_back(this->leds_.percentage);
  feedback->percentage.push_back(this->head_.percentage);
  feedback->percentage.push_back(this->left_arm_.percentage);
  feedback->percentage.push_back(this->right_arm_.percentage);

  //lock access to driver if necessary
  alg_.unlock();
}

///////////////////////////////////////////////////////////////////////////////
//   Loquendo TTS Action Client
///////////////////////////////////////////////////////////////////////////////
void TibiDaboHriAlgNode::ttsDone(const actionlib::SimpleClientGoalState& state,
                                 const iri_common_drivers_msgs::ttsResultConstPtr& result) 
{
  std_msgs::String expression;

  alg_.lock();

  ROS_INFO("TibiDaboHriAlgNode::ttsDone: Goal Achieved!"); 
  ROS_INFO("TibiDaboHriAlgNode::ttsDone: state=%s", state.toString().c_str()); 

  if( state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED )
    this->tts_.succeeded=true;
  else 
    this->tts_.succeeded=false;
    
  this->tts_.active=false;
  if(this->empty_leds_sequence_)
  {
    expression.data="stand_by";
    this->facial_expression_publisher_.publish(expression);
  }

  alg_.unlock();
} 

void TibiDaboHriAlgNode::ttsActive(void) 
{ 
  ROS_INFO("TibiDaboHriAlgNode::ttsActive: Goal just went active!"); 
} 

void TibiDaboHriAlgNode::ttsFeedback(const iri_common_drivers_msgs::ttsFeedbackConstPtr& feedback) 
{
  ROS_DEBUG("TibiDaboHriAlgNode::ttsFeedback: Got Feedback!"); 
  alg_.lock();
  if(feedback->busy)
    this->tts_.percentage=0.0;
  else
    this->tts_.percentage=100.0;
  alg_.unlock();
  ROS_DEBUG("TibiDaboHriAlgNode::ttsFeedback: busy=%d", feedback->busy); 
}

///////////////////////////////////////////////////////////////////////////////
//   Head Leds Action Client
///////////////////////////////////////////////////////////////////////////////
void TibiDaboHriAlgNode::ledsDone(const actionlib::SimpleClientGoalState& state,
                                  const tibi_dabo_msgs::sequenceResultConstPtr& result)
{
  alg_.lock();
  
  ROS_INFO("TibiDaboHriAlgNode::ledsDone: state=%s successful=%d", state.toString().c_str(), result->successful[0]);

  if( state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED )
    this->leds_.succeeded=true;
  else
    this->leds_.succeeded=false;

  this->leds_.active=false;
  alg_.unlock();
}

void TibiDaboHriAlgNode::ledsActive(void)
{
  ROS_INFO("TibiDaboHriAlgNode::ledsActive: Goal just went active!"); 
}

void TibiDaboHriAlgNode::ledsFeedback(const tibi_dabo_msgs::sequenceFeedbackConstPtr& feedback)
{
  ROS_DEBUG("TibiDaboHriAlgNode::ledsFeedback: percentage=%f", feedback->percentage[0]); 
  alg_.lock();
  this->leds_.percentage=feedback->percentage[0];
  alg_.unlock();
}

///////////////////////////////////////////////////////////////////////////////
//   Head Sequence Action Client
///////////////////////////////////////////////////////////////////////////////

void TibiDaboHriAlgNode::headDone(const actionlib::SimpleClientGoalState& state,
                                  const tibi_dabo_msgs::sequenceResultConstPtr& result)
{
  alg_.lock();
  
  ROS_INFO("TibiDaboHriAlgNode::headDone: state=%s successful=%d", state.toString().c_str(), result->successful[0]);

  if( state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED )
    this->head_.succeeded=true;
  else
    this->head_.succeeded=false;

  this->head_.active=false;
  alg_.unlock();
}

void TibiDaboHriAlgNode::headActive(void)
{
  ROS_INFO("TibiDaboHriAlgNode::headActive: Goal just went active!"); 
}

void TibiDaboHriAlgNode::headFeedback(const tibi_dabo_msgs::sequenceFeedbackConstPtr& feedback)
{
  ROS_DEBUG("TibiDaboHriAlgNode::headFeedback: percentage=%f", feedback->percentage[0]);
  alg_.lock();
  this->head_.percentage=feedback->percentage[0];
  alg_.unlock(); 
}

///////////////////////////////////////////////////////////////////////////////
//   Left Arm Sequence Action Client
///////////////////////////////////////////////////////////////////////////////
void TibiDaboHriAlgNode::leftArmDone(const actionlib::SimpleClientGoalState& state, 
                                     const tibi_dabo_msgs::sequenceResultConstPtr& result)
{
  alg_.lock();
  
  ROS_INFO("TibiDaboHriAlgNode::leftArmDone: state=%s successful=%d", state.toString().c_str(), result->successful[0]);

  if( state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED )
    this->left_arm_.succeeded=true;
  else
    this->left_arm_.succeeded=false;

  this->left_arm_.active=false;
  alg_.unlock();
}

void TibiDaboHriAlgNode::leftArmActive(void)
{
  ROS_INFO("TibiDaboHriAlgNode::leftArmActive: Goal just went active!"); 
}

void TibiDaboHriAlgNode::leftArmFeedback(const tibi_dabo_msgs::sequenceFeedbackConstPtr& feedback)
{
  alg_.lock();
  ROS_DEBUG("TibiDaboHriAlgNode::leftArmFeedback: percentage=%f", feedback->percentage[0]);
  this->left_arm_.percentage=feedback->percentage[0];
  alg_.unlock();
}

///////////////////////////////////////////////////////////////////////////////
//   Right Arm Sequence Action Client
///////////////////////////////////////////////////////////////////////////////
void TibiDaboHriAlgNode::rightArmDone(const actionlib::SimpleClientGoalState& state, 
                                      const tibi_dabo_msgs::sequenceResultConstPtr& result)
{
  alg_.lock();
  
  ROS_INFO("TibiDaboHriAlgNode::rightArmDone: state=%s successful=%d", state.toString().c_str(), result->successful[0]);

  if( state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED )
    this->right_arm_.succeeded=true;
  else
    this->right_arm_.succeeded=false;

  this->right_arm_.active=false;
  alg_.unlock();  
}

void TibiDaboHriAlgNode::rightArmActive(void)
{
  ROS_INFO("TibiDaboHriAlgNode::rightArmActive: Goal just went active!"); 
}

void TibiDaboHriAlgNode::rightArmFeedback(const tibi_dabo_msgs::sequenceFeedbackConstPtr& feedback)
{
  alg_.lock();
  ROS_DEBUG("TibiDaboHriAlgNode::rightArmFeedback: percentage=%f", feedback->percentage[0]);
  this->right_arm_.percentage=feedback->percentage[0];
  alg_.unlock();
}

/*  [action requests] */
void TibiDaboHriAlgNode::ttsMakeActionRequest(const iri_common_drivers_msgs::ttsGoal & tts_goal)
{
  ROS_INFO("TibiDaboHriAlgNode::ttsMakeActionRequest: Starting New Request!"); 

  //wait for the action server to start 
  //will wait for infinite time 
  this->tts_.client->waitForServer();  
  ROS_DEBUG("TibiDaboHriAlgNode::ttsMakeActionRequest: TTS Server is Available!"); 

  //send a goal to the action 
  tts_.client->sendGoal(tts_goal, 
             boost::bind(&TibiDaboHriAlgNode::ttsDone,     this, _1, _2), 
             boost::bind(&TibiDaboHriAlgNode::ttsActive,   this), 
             boost::bind(&TibiDaboHriAlgNode::ttsFeedback, this, _1)); 
  
  this->tts_.active=true;
}

void TibiDaboHriAlgNode::ledsMakeActionRequest(const tibi_dabo_msgs::sequenceGoal & leds_goal)
{
  ROS_INFO("TibiDaboHriAlgNode::ledsMakeActionRequest: Starting New Request!"); 
  this->leds_.client->waitForServer();  
  ROS_DEBUG("TibiDaboHriAlgNode::ledsMakeActionRequest: Leds Server is Available!"); 

  //send a goal to the action 
  this->leds_.client->sendGoal(leds_goal, 
              boost::bind(&TibiDaboHriAlgNode::ledsDone,     this, _1, _2), 
              boost::bind(&TibiDaboHriAlgNode::ledsActive,   this), 
              boost::bind(&TibiDaboHriAlgNode::ledsFeedback, this, _1)); 

  this->leds_.active=true;
}

void TibiDaboHriAlgNode::headMakeActionRequest(const tibi_dabo_msgs::sequenceGoal & head_goal)
{
  ROS_INFO("TibiDaboHriAlgNode::headMakeActionRequest: Starting New Request!"); 
  this->head_.client->waitForServer();  
  ROS_DEBUG("TibiDaboHriAlgNode::headMakeActionRequest: Head Server is Available!"); 

  //send a goal to the action 
  this->head_.client->sendGoal(head_goal, 
              boost::bind(&TibiDaboHriAlgNode::headDone,     this, _1, _2), 
              boost::bind(&TibiDaboHriAlgNode::headActive,   this), 
              boost::bind(&TibiDaboHriAlgNode::headFeedback, this, _1)); 

  this->head_.active=true;
}

void TibiDaboHriAlgNode::leftArmMakeActionRequest(const tibi_dabo_msgs::sequenceGoal & left_arm_goal)
{
  ROS_INFO("TibiDaboHriAlgNode::leftArmMakeActionRequest: Starting New Request!"); 
  this->left_arm_.client->waitForServer();  
  ROS_DEBUG("TibiDaboHriAlgNode::headMakeActionRequest: Head Server is Available!"); 

  //send a goal to the action 

  this->left_arm_.client->sendGoal(left_arm_goal, 
              boost::bind(&TibiDaboHriAlgNode::leftArmDone,     this, _1, _2), 
              boost::bind(&TibiDaboHriAlgNode::leftArmActive,   this), 
              boost::bind(&TibiDaboHriAlgNode::leftArmFeedback, this, _1)); 
  this->left_arm_.active=true;
}

void TibiDaboHriAlgNode::rightArmMakeActionRequest(const tibi_dabo_msgs::sequenceGoal & right_arm_goal)
{
  ROS_INFO("TibiDaboHriAlgNode::leftArmMakeActionRequest: Starting New Request!"); 
  this->right_arm_.client->waitForServer();  
  ROS_DEBUG("TibiDaboHriAlgNode::headMakeActionRequest: Head Server is Available!"); 

  //send a goal to the action 
  this->right_arm_.client->sendGoal(right_arm_goal, 
              boost::bind(&TibiDaboHriAlgNode::rightArmDone,     this, _1, _2), 
              boost::bind(&TibiDaboHriAlgNode::rightArmActive,   this), 
              boost::bind(&TibiDaboHriAlgNode::rightArmFeedback, this, _1)); 
  this->right_arm_.active=true;
}

void TibiDaboHriAlgNode::node_config_update(Config &config, uint32_t level)
{
}

void TibiDaboHriAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TibiDaboHriAlgNode>(argc, argv, "tibi_dabo_hri_alg_node");
}
