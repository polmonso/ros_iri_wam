#include "wam_move_arm_alg_node.h"

WamMoveArmAlgNode::WamMoveArmAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<WamMoveArmAlgorithm>(),
  move_arm_aserver_(public_node_handle_, "move_arm"),
  controller_client_("controller_client_", true),
  syn_move_arm_client_("syn_move_arm", true),
  send_msg_(false),
  final_state(2)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  /*syn_controller_aserver_.registerStartCallback(boost::bind(&WamMoveArmAlgNode::syn_controllerStartCallback, this, _1)); 
  syn_controller_aserver_.registerStopCallback(boost::bind(&WamMoveArmAlgNode::syn_controllerStopCallback, this)); 
  syn_controller_aserver_.registerIsFinishedCallback(boost::bind(&WamMoveArmAlgNode::syn_controllerIsFinishedCallback, this)); 
  syn_controller_aserver_.registerHasSucceedCallback(boost::bind(&WamMoveArmAlgNode::syn_controllerHasSucceedCallback, this)); 
  syn_controller_aserver_.registerGetResultCallback(boost::bind(&WamMoveArmAlgNode::syn_controllerGetResultCallback, this, _1)); 
  syn_controller_aserver_.registerGetFeedbackCallback(boost::bind(&WamMoveArmAlgNode::syn_controllerGetFeedbackCallback, this, _1)); 
  syn_controller_aserver_.start();*/
  move_arm_aserver_.registerStartCallback(boost::bind(&WamMoveArmAlgNode::move_armStartCallback, this, _1)); 
  move_arm_aserver_.registerStopCallback(boost::bind(&WamMoveArmAlgNode::move_armStopCallback, this)); 
  move_arm_aserver_.registerIsFinishedCallback(boost::bind(&WamMoveArmAlgNode::move_armIsFinishedCallback, this)); 
  move_arm_aserver_.registerHasSucceedCallback(boost::bind(&WamMoveArmAlgNode::move_armHasSucceedCallback, this)); 
  move_arm_aserver_.registerGetResultCallback(boost::bind(&WamMoveArmAlgNode::move_armGetResultCallback, this, _1)); 
  move_arm_aserver_.registerGetFeedbackCallback(boost::bind(&WamMoveArmAlgNode::move_armGetFeedbackCallback, this, _1)); 
  move_arm_aserver_.start();
  
    action_server_follow_.reset(new FJTAS(public_node_handle_, "syn_controller",
                                        boost::bind(&WamMoveArmAlgNode::goalCBFollow, this, _1),
                                        boost::bind(&WamMoveArmAlgNode::cancelCBFollow, this, _1)));

  
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
/*
void WamMoveArmAlgNode::syn_controllerStartCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{ 
  alg_.lock(); 
    //check goal 
    //feedback_controller.
    //feedback_controller
    tmp_msg_= *goal;
    send_msg_=true;
    finish=false;
    success_contr=false;
    makeMsg(tmp_msg_);
    cliente_goal_=tmp_msg_;
    clienteMakeActionRequest();
  
    //execute goal 
    
  alg_.unlock(); 
} 


void WamMoveArmAlgNode::syn_controllerStopCallback(void) 
{ 
  alg_.lock(); 
    //stop action 
    send_msg_=false;
    clienteMakeActionRequest();
  alg_.unlock(); 
} 

bool WamMoveArmAlgNode::syn_controllerIsFinishedCallback(void) 
{ 
  bool ret = false; 

  alg_.lock(); 
    //if action has finish for any reason 

    //ret = true; 
  alg_.unlock(); 

  return ret; 
} 

bool WamMoveArmAlgNode::syn_controllerHasSucceedCallback(void) 
{ 
  bool ret = false; 

  alg_.lock(); 
    //if goal was accomplished 

    //ret = true 
  alg_.unlock(); 

  return ret; 
} 

void WamMoveArmAlgNode::syn_controllerGetResultCallback(control_msgs::FollowJointTrajectoryResultPtr& result) 
{ 
  alg_.lock(); 
    //update result data to be sent to client 
    //result->data = data; 

  alg_.unlock(); 
} 

void WamMoveArmAlgNode::syn_controllerGetFeedbackCallback(control_msgs::FollowJointTrajectoryFeedbackPtr& feedback) 
{ 
  alg_.lock(); 
    //keep track of feedback 
    //ROS_INFO("feedback: %s", feedback->data.c_str()); 

  alg_.unlock(); 
}
* */
void WamMoveArmAlgNode::syn_move_armDone(const actionlib::SimpleClientGoalState& state,  const arm_navigation_msgs::MoveArmResultConstPtr& result) 
{ 
  if( state.toString().compare("SUCCEEDED") == 0 ) 
    ROS_INFO("WamMoveArmAlgNode::syn_move_armDone: Goal Achieved!"); 
    
  else 
    ROS_INFO("WamMoveArmAlgNode::syn_move_armDone: %s", state.toString().c_str()); 

 ROS_ERROR_STREAM(""<<result->error_code.val<<"##");
  final_state= result->error_code.val;
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
  move_feedback.state= feedback->state;
  ROS_ERROR_STREAM("@"<<feedback->state);
  if(move_feedback.state == "RECALLING")feedback_is_ok=false;
  if(move_feedback.state == "PREEMPTING")feedback_is_ok=false;
  //if feedback is not what expected, cancel requested goal 
  if( !feedback_is_ok ) 
  { 
    syn_move_arm_client_.cancelGoal(); 
    ROS_INFO("WamMoveArmAlgNode::syn_move_armFeedback: Cancelling Action!"); 
  } 
  else move_feedback.time_to_completion= feedback->time_to_completion;
}
void WamMoveArmAlgNode::move_armStartCallback(const arm_navigation_msgs::MoveArmGoalConstPtr& goal)
{   move_feedback.state="PENDING";
    final_state=2;
    alg_.lock(); 
    move_arm(goal);
    
    syn_move_armMakeActionRequest(*goal);
    
    move_feedback.time_to_completion =this->alg_.getTime();
    alg_.unlock(); 
} 

void WamMoveArmAlgNode::move_armStopCallback(void) 
{ 
  alg_.lock(); 
    //stop action 
  alg_.unlock(); 
} 

bool WamMoveArmAlgNode::move_armIsFinishedCallback(void) 
{ 
  bool ret = false; 

  alg_.lock(); 
    //if action has finish for any reason 

   if( (move_feedback.state==" RECALLED") ||
       (move_feedback.state=="REJECTED" ) ||
       (move_feedback.state=="PREEMPTED") ||
       (move_feedback.state=="ABORTED"  ) ||
       (move_feedback.state=="SUCCEEDED") ||
       (move_feedback.state=="LOST"     ) )
        ret =  true;
   else if (final_state==1) move_feedback.state="SUCCEEDED";
   else if (final_state<=0) move_feedback.state="ABORTED";
   else  ret =  false;

  alg_.unlock(); 

  return ret; 
} 

bool WamMoveArmAlgNode::move_armHasSucceedCallback(void) 
{ 
  bool ret = false; 

  alg_.lock(); 
    //if goal was accomplished 
    if(move_feedback.state=="SUCCEEDED") ret=true;
  alg_.unlock(); 

  return ret; 
} 

void WamMoveArmAlgNode::move_armGetResultCallback(arm_navigation_msgs::MoveArmResultPtr& result) 
{ 
  alg_.lock(); 
    //update result data to be sent to client
      if(final_state == 1) result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS;
      else if(final_state == 0)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::GOAL_CONSTRAINTS_VIOLATED;
	  else if(final_state == -1)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::PLANNING_FAILED;
	  else if(final_state == -2)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::TIMED_OUT;
	  else if(final_state == -3)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::START_STATE_IN_COLLISION;
	  else if(final_state == -4)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS;
	  else if(final_state == -5)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::GOAL_IN_COLLISION;
	  else if(final_state == -6)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS;
	  else if(final_state == -7)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_ROBOT_STATE;
	  else if(final_state == -8)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INCOMPLETE_ROBOT_STATE;
	  else if(final_state == -9)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_PLANNER_ID;
	  else if(final_state == -10)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_NUM_PLANNING_ATTEMPTS;
	  else if(final_state == -11)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_ALLOWED_PLANNING_TIME;
	  else if(final_state == -12)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_GROUP_NAME;
	  else if(final_state == -13)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_GOAL_JOINT_CONSTRAINTS;
	  else if(final_state == -14)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_GOAL_POSITION_CONSTRAINTS;
	  else if(final_state == -15)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_GOAL_ORIENTATION_CONSTRAINTS;
	  else if(final_state == -16)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_PATH_JOINT_CONSTRAINTS;
	  else if(final_state == -17)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_PATH_POSITION_CONSTRAINTS;
	  else if(final_state == -18)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_PATH_ORIENTATION_CONSTRAINTS;
	  else if(final_state == -19)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_TRAJECTORY;
	  else if(final_state == -20)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_INDEX;
	  else if(final_state == -21)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::JOINT_LIMITS_VIOLATED;
	  else if(final_state == -22)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::PATH_CONSTRAINTS_VIOLATED;
	  else if(final_state == -23)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::COLLISION_CONSTRAINTS_VIOLATED;
	  else if(final_state == -24)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::GOAL_CONSTRAINTS_VIOLATED;
	  else if(final_state == -25)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::JOINTS_NOT_MOVING;
	  else if(final_state == -26)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::TRAJECTORY_CONTROLLER_FAILED;
	  else if(final_state == -27)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::FRAME_TRANSFORM_FAILURE;
	  else if(final_state == -28)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::COLLISION_CHECKING_UNAVAILABLE;
	  else if(final_state == -29)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::ROBOT_STATE_STALE;
	  else if(final_state == -30)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::SENSOR_INFO_STALE;
	  else if(final_state == -31)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::NO_IK_SOLUTION;
	  else if(final_state == -32)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_LINK_NAME;
	  else if(final_state == -33)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::IK_LINK_IN_COLLISION;
	  else if(final_state == -34)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::NO_FK_SOLUTION;
	  else if(final_state == -35)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::KINEMATICS_STATE_IN_COLLISION;
	  else if(final_state == -36)result->error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_TIMEOUT;

  // if(final_state == "SUCCEEDED") result->error_code = 1;
    //*result=move_result;
    
    //result->data = data; 
  alg_.unlock(); 
} 

void WamMoveArmAlgNode::move_armGetFeedbackCallback(arm_navigation_msgs::MoveArmFeedbackPtr& feedback) 
{ 
  alg_.lock(); 
    //keep track of feedback 
  
    feedback->state=move_feedback.state;
    feedback->time_to_completion= move_feedback.time_to_completion;
  alg_.unlock(); 
}

void WamMoveArmAlgNode::clienteDone(const actionlib::SimpleClientGoalState& state,  const control_msgs::FollowJointTrajectoryResultConstPtr& result) 
{ ROS_WARN("HOLA$$$$");
  if( state.toString().compare("SUCCEEDED") == 0 ) 
  {
    ROS_INFO("WamMoveArmAlgNode::clienteDone: Goal Achieved!"); 
    goal_h.setSucceeded();
    final_state=0;
  }
  else 
  {
    ROS_INFO("WamMoveArmAlgNode::clienteDone: %s ", state.toString().c_str()); 
 	 if( (state.toString().compare("ABORTED ") == 0 ) ||
 	     (state.toString().compare("LOST") == 0 ) ||
 	     (state.toString().compare("PREEMPTED ") == 0 ) )
 	     {
			goal_h.setAborted();
		 }
     else if( (state.toString().compare("RECALLED ") == 0 ) ||
			  (state.toString().compare("REJECTED ") == 0 ) ||
			  (state.toString().compare("CANCELED ") == 0 ) )
		 {
			 goal_h.setCanceled();
		 }
			 
  }
 
} 
void WamMoveArmAlgNode::f(const actionlib::SimpleClientGoalState& state)
{
	 ROS_WARN_STREAM("HOLA$$$$"<<state.toString());
}

void WamMoveArmAlgNode::clienteActive() 
{ 
  //ROS_INFO("WamMoveArmAlgNode::clienteActive: Goal just went active!"); 
} 

void WamMoveArmAlgNode::clienteFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback) 
{ 
  ROS_INFO("WamMoveArmAlgNode::clienteFeedback: Got Feedback!"); 

  bool feedback_is_ok = true; 
 //ROS_WARN_STREAM("Desde Cleinte:"<<state.toString().c_str());
  //analyze feedback 
  //my_var = feedback->var; 
//feedback_controller=*feedback;
ROS_WARN_STREAM("$$"<<feedback->desired<<"");
ROS_WARN_STREAM("$$"<<feedback->actual<<"");

  //if feedback is not what expected, cancel requested goal 
  if( !feedback_is_ok ) 
  { 
    controller_client_.cancelGoal(); 
    //ROS_INFO("WamMoveArmAlgNode::clienteFeedback: Cancelling Action!"); 
  } 
}

/*  [action requests] */
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
void WamMoveArmAlgNode::clienteMakeActionRequest() 
{ 
  ROS_INFO("WamMoveArmAlgNode::clienteMakeActionRequest: Starting New Request!"); 

  //wait for the action server to start 
  //will wait for infinite time 
  controller_client_.waitForServer();  
  ROS_INFO("WamMoveArmAlgNode::clienteMakeActionRequest: Server is Available!"); 
  controller_client_.sendGoal(cliente_goal_, 
              boost::bind(&WamMoveArmAlgNode::clienteDone,     this, _1, _2), 
              boost::bind(&WamMoveArmAlgNode::clienteActive,   this), 
              boost::bind(&WamMoveArmAlgNode::clienteFeedback, this, _1));
             
  ROS_INFO("WamMoveArmAlgNode::clienteMakeActionRequest: Goal Sent. Wait for Result!"); 
}

void WamMoveArmAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void WamMoveArmAlgNode::addNodeDiagnostics(void)
{
}
void WamMoveArmAlgNode::makeMsg(control_msgs::FollowJointTrajectoryGoal& msg)
{
	if(send_msg_)
	{
		trajectory_msgs::JointTrajectory tmp_traj=msg.trajectory;
		//this->alg_.restoreTime(tmp_traj,this->alg_.getTime());
		this->alg_.restoreVelocity(tmp_traj,this->alg_.getTime());
	//	this->alg_.restoreAccel(tmp_traj,this->alg_.getTime());
		msg.trajectory=tmp_traj;		
		send_msg_=false;
		//ROS_WARN_STREAM(""<<tmp_traj);
	}
	else
	{
		control_msgs::FollowJointTrajectoryGoal empty_msg;
		empty_msg.trajectory.joint_names.push_back("j1_joint");
		empty_msg.trajectory.joint_names.push_back("j2_joint");
		empty_msg.trajectory.joint_names.push_back("j3_joint");
		empty_msg.trajectory.joint_names.push_back("j4_joint");
		empty_msg.trajectory.joint_names.push_back("j5_joint");
		empty_msg.trajectory.joint_names.push_back("j6_joint");
		empty_msg.trajectory.joint_names.push_back("j7_joint");
		empty_msg.trajectory.points.resize(1);
		empty_msg.trajectory.points[0].positions.resize(7);
		for(int i=0; i < 7; ++i)empty_msg.trajectory.points[0].positions[i]=0;
		
		msg=empty_msg;
	}
	
}
void WamMoveArmAlgNode::move_arm(const arm_navigation_msgs::MoveArmGoalConstPtr& goal)
{
	getTime(*goal,this->alg_.path_time_);
}

void WamMoveArmAlgNode::getTime(const arm_navigation_msgs::MoveArmGoal& msg, ros::Duration& time)
{
 time = msg.motion_plan_request.expected_path_dt;

}
void WamMoveArmAlgNode::goalCBFollow(GoalHandle goal)
{
	alg_.lock(); 
    tmp_msg_= *(goal.getGoal());
    goal_h=goal;
    goal.setAccepted();
    send_msg_=true;
//    finish=false;
 //   success_contr=false;
    makeMsg(tmp_msg_);
    cliente_goal_=tmp_msg_;
    clienteMakeActionRequest();
    //execute goal 
    alg_.unlock(); 
}
void WamMoveArmAlgNode::cancelCBFollow(GoalHandle goal){}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<WamMoveArmAlgNode>(argc, argv, "wam_move_arm_alg_node");
}
