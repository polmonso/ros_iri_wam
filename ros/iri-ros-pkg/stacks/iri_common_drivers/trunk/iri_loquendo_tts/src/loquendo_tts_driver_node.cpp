#include "loquendo_tts_driver_node.h"

LoquendoTtsDriverNode::LoquendoTtsDriverNode(ros::NodeHandle &nh) : iri_base_driver::IriBaseNodeDriver<LoquendoTtsDriver>(nh),
  tts_aserver_(this->public_node_handle_, "tts")
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  tts_aserver_.registerStartCallback(boost::bind(&LoquendoTtsDriverNode::startCallback, this, _1));
  tts_aserver_.registerStopCallback(boost::bind(&LoquendoTtsDriverNode::stopCallback, this));
  tts_aserver_.registerIsFinishedCallback(boost::bind(&LoquendoTtsDriverNode::isFinishedCallback, this));
  tts_aserver_.registerHasSucceedCallback(boost::bind(&LoquendoTtsDriverNode::hasSucceedCallback, this));
  tts_aserver_.registerGetResultCallback(boost::bind(&LoquendoTtsDriverNode::getResultCallback, this, _1));
  tts_aserver_.registerGetFeedbackCallback(boost::bind(&LoquendoTtsDriverNode::getFeedbackCallback, this, _1));
  
  // [init action clients]
}

void LoquendoTtsDriverNode::startCallback(const iri_common_driver_driver_msgs::ttsGoalConstPtr& goal)
{
  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
    driver_.playSpeech(goal->msg);

  //unlock access to driver if necessary
  this->driver_.unlock();
  
//   return accept_goal;
}

void LoquendoTtsDriverNode::stopCallback(void)
{
  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
    driver_.stopSpeech();
  
  //lock access to driver if necessary
  this->driver_.unlock();
}

bool LoquendoTtsDriverNode::isFinishedCallback(void)
{
  bool ret = false;
  
  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
    ret = !driver_.isBusy();
  
  //lock access to driver if necessary
  this->driver_.unlock();

  return ret;
}

bool LoquendoTtsDriverNode::hasSucceedCallback(void)
{
  bool ret = false;
    
  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
    ret = true;
  
  //lock access to driver if necessary
  this->driver_.unlock();
  
  return ret;
}

void LoquendoTtsDriverNode::getResultCallback(iri_common_driver_msgs::ttsResultPtr& result)
{
  if( driver_.isRunning() )
    result->ok = hasSucceedCallback();
  else
    result->ok = false;
}

void LoquendoTtsDriverNode::getFeedbackCallback(iri_common_driver_msgs::ttsFeedbackPtr& feedback)
{
  //lock access to driver if necessary
  this->driver_.lock();

  if( driver_.isRunning() )
    feedback->busy = driver_.isBusy();
  
  //lock access to driver if necessary
  this->driver_.unlock();
}

void LoquendoTtsDriverNode::mainNodeThread(void)
{
  //lock access to driver if necessary
  this->driver_.lock();

  // [fill msg Header if necessary]
  //<publisher_name>.header.stamp = ros::Time::now();
  //<publisher_name>.header.frame_id = "<publisher_topic_name>";

  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
//   ROS_INFO("TTS Loquendo: Busy=%d",driver_.isBusy());

  //unlock access to driver if previously blocked
  this->driver_.unlock();
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void LoquendoTtsDriverNode::postNodeOpenHook(void)
{
  tts_aserver_.start();
  ROS_INFO("LoquendoTtsDriverNode:: Server Started!"); 
}

void LoquendoTtsDriverNode::addNodeDiagnostics(void)
{
}

void LoquendoTtsDriverNode::addNodeOpenedTests(void)
{
}

void LoquendoTtsDriverNode::addNodeStoppedTests(void)
{
}

void LoquendoTtsDriverNode::addNodeRunningTests(void)
{
}

void LoquendoTtsDriverNode::reconfigureNodeHook(int level)
{
}

LoquendoTtsDriverNode::~LoquendoTtsDriverNode()
{
  // [free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<LoquendoTtsDriverNode>(argc, argv, "loquendo_server");
}
