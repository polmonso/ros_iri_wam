#include "wam_arm_navigation_alg_node.h"

WamArmNavigationAlgNode::WamArmNavigationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<WamArmNavigationAlgorithm>(),
  aborted(false),
  simple_pose_move_aserver_(public_node_handle_, "simple_pose_move"),
  move_iri_wam_client_("move_iri_wam", true),
  state_msg(""),
  state__("")
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  this->simple_pose_topic_subscriber_ = this->public_node_handle_.subscribe("simple_pose_topic", 100, &WamArmNavigationAlgNode::simple_pose_topic_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  simple_pose_move_aserver_.registerStartCallback(boost::bind(&WamArmNavigationAlgNode::simple_pose_moveStartCallback, this, _1)); 
  simple_pose_move_aserver_.registerStopCallback(boost::bind(&WamArmNavigationAlgNode::simple_pose_moveStopCallback, this)); 
  simple_pose_move_aserver_.registerIsFinishedCallback(boost::bind(&WamArmNavigationAlgNode::simple_pose_moveIsFinishedCallback, this)); 
  simple_pose_move_aserver_.registerHasSucceedCallback(boost::bind(&WamArmNavigationAlgNode::simple_pose_moveHasSucceedCallback, this)); 
  simple_pose_move_aserver_.registerGetResultCallback(boost::bind(&WamArmNavigationAlgNode::simple_pose_moveGetResultCallback, this, _1)); 
  simple_pose_move_aserver_.registerGetFeedbackCallback(boost::bind(&WamArmNavigationAlgNode::simple_pose_moveGetFeedbackCallback, this, _1)); 
  simple_pose_move_aserver_.start();
  
  // [init action clients]
}

WamArmNavigationAlgNode::~WamArmNavigationAlgNode(void)
{
  // [free dynamic memory]
}

void WamArmNavigationAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]
//  move_iri_wamMakeActionRequest();

  // [publish messages]
}

/*  [subscriber callbacks] */
void WamArmNavigationAlgNode::simple_pose_topic_callback(const iri_wam_arm_navigation::PoseSimple::ConstPtr& msg) 
{ 
  ROS_INFO("WamArmNavigationAlgNode::simple_pose_topic_callback: New Message Received"); 

   
  //use appropiate mutex to shared variables if necessary 
  this->alg_.lock(); 
  //this->simple_pose_topic_mutex_.enter(); 
    this->alg_.setTarget(msg->goal,msg->header.frame_id);
    arm_navigation_msgs::MoveArmGoal move_arm;
    makeMoveMsg(move_arm);
    move_iri_wamMakeActionRequest(move_arm);
  //std::cout << msg->data << std::endl; 

  //unlock previously blocked shared variables 
  this->alg_.unlock(); 
  //this->simple_pose_topic_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */
void WamArmNavigationAlgNode::move_iri_wamDone(const actionlib::SimpleClientGoalState& state,  const arm_navigation_msgs::MoveArmResultConstPtr& result) 
{ 
  
  ROS_INFO("WamArmNavigationAlgNode::move_iri_wamDone: %s", state.toString().c_str()); 
  if( state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED ) 
  {
    aborted=false;
    state__="FREE";
  }
  else 
  {
    aborted=true;
    state__="ABORTED";
  }
  code_result=result->error_code;
  //copy & work with requested result 
} 

void WamArmNavigationAlgNode::move_iri_wamActive() 
{ 
  //ROS_INFO("WamArmNavigationAlgNode::move_iri_wamActive: Goal just went active!"); 
} 

void WamArmNavigationAlgNode::move_iri_wamFeedback(const arm_navigation_msgs::MoveArmFeedbackConstPtr& feedback) 
{ 
  //ROS_INFO("WamArmNavigationAlgNode::move_iri_wamFeedback: Got Feedback!"); 

  bool feedback_is_ok = true; 

  //analyze feedback 
  //my_var = feedback->var; 
  state_msg= feedback->state;
  //if feedback is not what expected, cancel requested goal 
  if( !feedback_is_ok ) 
  { 
    move_iri_wam_client_.cancelGoal(); 
    //ROS_INFO("WamArmNavigationAlgNode::move_iri_wamFeedback: Cancelling Action!"); 
  } 
}

void WamArmNavigationAlgNode::simple_pose_moveStartCallback(const iri_wam_arm_navigation::SimplePoseGoalConstPtr& goal)
{ 
    alg_.lock(); 
    //check goal 
    alg_.setTarget(goal->goal,goal->frame_id);
    arm_navigation_msgs::MoveArmGoal move_arm;
    makeMoveMsg(move_arm);
    move_iri_wamMakeActionRequest(move_arm);
    
    state__="NO FREE";
    aborted=false;
    // Show the goal Pose
    ROS_INFO_STREAM(goal->goal);
    //execute goal 
    alg_.unlock(); 
} 

void WamArmNavigationAlgNode::simple_pose_moveStopCallback(void) 
{ 
  alg_.lock(); 
    //stop action 
    arm_navigation_msgs::MoveArmGoal move_arm;
    makeMoveMsg(move_arm);
    move_iri_wamMakeActionRequest(move_arm);
  alg_.unlock(); 
} 

bool WamArmNavigationAlgNode::simple_pose_moveIsFinishedCallback(void) 
{ 
  bool ret = false; 
  alg_.lock(); 
    //if action has finish for any reason 
    if( (state_msg.compare("SUCCEEDED") == 0 )  ||
		(state_msg.compare("LOST") == 0 ) || 
		(state_msg.compare("ABORTED") == 0 ) ||
		(state_msg.compare("FAILED") == 0 ) )
		{
         ret = true; 
		}
  alg_.unlock(); 

  return ret; 
} 

bool WamArmNavigationAlgNode::simple_pose_moveHasSucceedCallback(void) 
{ 
  bool ret = false; 

  alg_.lock(); 
    //if goal was accomplished 
    if(code_result.val==arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS){      
      ret=true;
    }
      
    //ret = true 
  alg_.unlock(); 

  return ret; 
} 

void WamArmNavigationAlgNode::simple_pose_moveGetResultCallback(iri_wam_arm_navigation::SimplePoseResultPtr& result) 
{ 
  alg_.lock(); 
    //update result data to be sent to client 
    (*result).error_code=code_result;
    //result->data = data; 
  alg_.unlock(); 
} 

void WamArmNavigationAlgNode::simple_pose_moveGetFeedbackCallback(iri_wam_arm_navigation::SimplePoseFeedbackPtr& feedback) 
{ 
  alg_.lock(); 
    //    ROS_INFO("feedback: %s", state_msg.c_str());
    //keep track of feedback 

//  (*feedback).state=state_msg;
  if (state__.compare("FREE")==0){
    (*feedback).state="FREE";
  }
  else if (state__.compare("ABORTED")==0){
    (*feedback).state="ABORTED";
  }
  else (*feedback).state="NO FREE";

  (*feedback).succesed=(state_msg.compare("SUCCEEDED"))?true:false; 
  alg_.unlock(); 
}

/*  [action requests] */
void WamArmNavigationAlgNode::move_iri_wamMakeActionRequest(const arm_navigation_msgs::MoveArmGoal& msg) 
{ 
  ROS_INFO("WamArmNavigationAlgNode::move_iri_wamMakeActionRequest: Starting New Request!"); 

  //wait for the action server to start 
  //will wait for infinite time 
  move_iri_wam_client_.waitForServer();  
  ROS_INFO("WamArmNavigationAlgNode::move_iri_wamMakeActionRequest: Server is Available!"); 

  //send a goal to the action 
  //move_iri_wam_goal_.data = my_desired_goal; 
  move_iri_wam_client_.sendGoal(msg, 
              boost::bind(&WamArmNavigationAlgNode::move_iri_wamDone,     this, _1, _2), 
              boost::bind(&WamArmNavigationAlgNode::move_iri_wamActive,   this), 
              boost::bind(&WamArmNavigationAlgNode::move_iri_wamFeedback, this, _1)); 
  this->alg_.sendedPose();
  ROS_INFO("WamArmNavigationAlgNode::move_iri_wamMakeActionRequest: Goal Sent. Wait for Result!"); 
  // Waiting for the result
  //move_iri_wam_client_.waitForResult();
}

void WamArmNavigationAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void WamArmNavigationAlgNode::addNodeDiagnostics(void)
{
}
void WamArmNavigationAlgNode::makeMoveMsg(arm_navigation_msgs::MoveArmGoal& goal)
{
	if(this->alg_.hasPose())
	{
		this->alg_.setPlannerRequest(goal);
		this->alg_.setPlannerRequestPose(goal);
	}
	else 
	{
		arm_navigation_msgs::MoveArmGoal empty;
		goal=empty;
	}
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<WamArmNavigationAlgNode>(argc, argv, "wam_arm_navigation_alg_node");
  
}
