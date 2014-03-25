#include "wam_move_arm_alg_node.h"

WamMoveArmAlgNode::WamMoveArmAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<WamMoveArmAlgorithm>(),
  has_move_arm_feedback(false),
  move_arm_aserver_(public_node_handle_, "move_arm"),
  syn_move_arm_client_("syn_move_arm", true)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  move_arm_aserver_.registerStartCallback(boost::bind(&WamMoveArmAlgNode::move_armStartCallback, this, _1)); 
  move_arm_aserver_.registerStopCallback(boost::bind(&WamMoveArmAlgNode::move_armStopCallback, this)); 
  move_arm_aserver_.registerIsFinishedCallback(boost::bind(&WamMoveArmAlgNode::move_armIsFinishedCallback, this)); 
  move_arm_aserver_.registerHasSucceedCallback(boost::bind(&WamMoveArmAlgNode::move_armHasSucceedCallback, this)); 
  move_arm_aserver_.registerGetResultCallback(boost::bind(&WamMoveArmAlgNode::move_armGetResultCallback, this, _1)); 
  move_arm_aserver_.registerGetFeedbackCallback(boost::bind(&WamMoveArmAlgNode::move_armGetFeedbackCallback, this, _1)); 
  move_arm_aserver_.start();

  private_node_handle_.param<double>("tool_x", this->alg_.tool_x, 0.0);
  private_node_handle_.param<double>("tool_y", this->alg_.tool_y, 0.0);
  private_node_handle_.param<double>("tool_z", this->alg_.tool_z, 0.0);

  // [init action clients]
}

WamMoveArmAlgNode::~WamMoveArmAlgNode(void)
{
  // [free dynamic memory]
}

void WamMoveArmAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]
  //syn_move_armMakeActionRequest();
  //clienteMakeActionRequest();

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

void WamMoveArmAlgNode::syn_move_armDone(const actionlib::SimpleClientGoalState& state,  const arm_navigation_msgs::MoveArmResultConstPtr& result) 
{ 
  ROS_INFO("WamMoveArmAlgNode::syn_move_armDone: %s", state.toString().c_str()); 

  error_code_move_arm=result->error_code;
  contact_information_move_arm=result->contacts;
  state_move_arm_goal=state.toString();
  //copy & work with requested result 
} 

void WamMoveArmAlgNode::syn_move_armActive() 
{ 
  //ROS_INFO("WamMoveArmAlgNode::syn_move_armActive: Goal just went active!"); 
} 

void WamMoveArmAlgNode::syn_move_armFeedback(const arm_navigation_msgs::MoveArmFeedbackConstPtr& feedback) 
{ 
  //ROS_INFO_STREAM("WamMoveArmAlgNode::syn_move_armFeedback: Got Feedback!"<<feedback->state); 
  bool feedback_is_ok = true; 

  //analyze feedback 
  //my_var = feedback->var; 
  has_move_arm_feedback = true;
  state_move_arm_feedback = feedback->state;
  time_move_arm_feedback = feedback->time_to_completion;
  //if feedback is not what expected, cancel requested goal 
  if( !feedback_is_ok ) 
  { 
    syn_move_arm_client_.cancelGoal(); 
    ROS_INFO("WamMoveArmAlgNode::syn_move_armFeedback: Cancelling Action!"); 
  } 
  else 
    move_feedback.time_to_completion = feedback->time_to_completion;
}
void WamMoveArmAlgNode::syn_move_armMakeActionRequest(const arm_navigation_msgs::MoveArmGoal& goal) 
{ 
  ROS_INFO("WamMoveArmAlgNode::syn_move_armMakeActionRequest: Starting New Request!"); 

  //wait for the action server to start 
  //will wait for infinite time 
  syn_move_arm_client_.waitForServer();  
  ROS_INFO("WamMoveArmAlgNode::syn_move_armMakeActionRequest: Server is Available!"); 
	
  //send a goal to the action 
  syn_move_arm_goal_ = goal; 
  syn_move_arm_client_.sendGoal(syn_move_arm_goal_, 
              boost::bind(&WamMoveArmAlgNode::syn_move_armDone,     this, _1, _2), 
              boost::bind(&WamMoveArmAlgNode::syn_move_armActive,   this), 
              boost::bind(&WamMoveArmAlgNode::syn_move_armFeedback, this, _1)); 
  ROS_INFO("WamMoveArmAlgNode::syn_move_armMakeActionRequest: Goal Sent. Wait for Result!"); 
}
void WamMoveArmAlgNode::move_armStartCallback(const arm_navigation_msgs::MoveArmGoalConstPtr& goal)
{   
	move_feedback.state="PENDING";
    alg_.lock(); 
    arm_navigation_msgs::MoveArmGoal mov = *goal;
    this->alg_.reconfigure(mov);
    syn_move_armMakeActionRequest(mov);    
    alg_.unlock(); 
} 
void WamMoveArmAlgNode::move_armStopCallback(void) 
{ 
  alg_.lock(); 
    //stop action 
    arm_navigation_msgs::MoveArmGoal empty;
    syn_move_armMakeActionRequest(empty);
  alg_.unlock(); 
} 
bool WamMoveArmAlgNode::move_armIsFinishedCallback(void) 
{ 
  bool ret = false; 

  alg_.lock(); 
    //if action has finish for any reason 
   if ( (state_move_arm_goal.compare("SUCCEEDED") == 0) ||
		(state_move_arm_goal.compare("LOST") == 0) ||
		(state_move_arm_goal.compare("ABORTED")== 0))
		{
			ret = true; 
			has_move_arm_feedback=false;
		}
  alg_.unlock(); 

  return ret; 
} 

bool WamMoveArmAlgNode::move_armHasSucceedCallback(void) 
{ 
  bool ret = false; 

  alg_.lock(); 
    //if goal was accomplished 
    if( state_move_arm_goal.compare("SUCCEEDED") == 0) ret=true;
  alg_.unlock(); 

  return ret; 
} 

void WamMoveArmAlgNode::move_armGetResultCallback(arm_navigation_msgs::MoveArmResultPtr& result) 
{ 
  alg_.lock(); 
    //update result data to be sent to client 
    (*result).error_code=error_code_move_arm;
    (*result).contacts=contact_information_move_arm;
    //result->data = data; 
  alg_.unlock(); 
} 

void WamMoveArmAlgNode::move_armGetFeedbackCallback(arm_navigation_msgs::MoveArmFeedbackPtr& feedback) 
{ 
  alg_.lock(); 
    //keep track of feedback 
    if(has_move_arm_feedback)
    {
		(*feedback).state=state_move_arm_feedback;
		(*feedback).time_to_completion=time_move_arm_feedback;		
	}
	else
	{
		(*feedback).state="WAIT FOR RESULT";
	}
    //ROS_INFO("feedback: %s", feedback->data.c_str()); 
  alg_.unlock(); 
}

void WamMoveArmAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void WamMoveArmAlgNode::addNodeDiagnostics(void)
{
}
/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<WamMoveArmAlgNode>(argc, argv, "wam_move_arm_alg_node");
}
