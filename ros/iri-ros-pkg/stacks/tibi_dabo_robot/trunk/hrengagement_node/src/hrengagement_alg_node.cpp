#include "hrengagement_alg_node.h"

HrengagementAlgNode::HrengagementAlgNode(void) :
  hri_client_("hri", true),
  MoveBase_client_("MoveBase", true),
  tf_listener_(ros::Duration(30.f)),
  track_people_(false)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  people_tracker_subscriber_ = public_node_handle_.subscribe("people_tracker", 100, &HrengagementAlgNode::people_tracker_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]

  // init hri goal request
  hri_goal_.sequence_file   = std::vector<std::string>(5, std::string(""));
  HRIMakeActionRequest(alg_.loadHRISequence("home.xml"));
//   hri_goal_.num_repetitions = std::vector<int>(5, 0);
}

HrengagementAlgNode::~HrengagementAlgNode(void)
{
  // [free dynamic memory]
}

void HrengagementAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  
//   if( state_ != e30)
//   {
//     state_ = alg_.hre_update(person_dist_, person_angle_);
// 
//     ROS_INFO("mainNodeThread: state=%i dist=%f", state_, person_dist_);
//     person_dist_-=0.1;
//   }
}

/*  [subscriber callbacks] */
void HrengagementAlgNode::people_tracker_callback(const iri_people_tracking::peopleTrackingArray::ConstPtr& msg) 
{
  //use appropiate mutex to shared variables if necessary 
  people_tracker_mutex_.enter(); 
  
//     ROS_INFO("HrengagementAlgNode::people_tracker_callback: track_people_=%d",track_people_);
    for(unsigned int ii=0; ii<msg->peopleSet.size(); ii++)
    {
      ROS_INFO("targetId=%d pos(%f,%f) vel(%f,%f)", msg->peopleSet[ii].targetId, 
               msg->peopleSet[ii].x, msg->peopleSet[ii].y, 
               msg->peopleSet[ii].vx, msg->peopleSet[ii].vy);
    }
    
    //if tracking people required
    if(track_people_)
    {
      unsigned int target_person = 0;
      if( !alg_.isSomeoneStanding(msg, target_person) )
      {
        ROS_WARN("HrengagementAlgNode::No people to track...");
      }
      else
      {
        goal_person_.targetId    = msg->peopleSet[target_person].targetId;
        goal_person_.x           = msg->peopleSet[target_person].x;
        goal_person_.y           = msg->peopleSet[target_person].y;
        goal_person_.vx          = msg->peopleSet[target_person].vx;
        goal_person_.vy          = msg->peopleSet[target_person].vy;
        goal_person_.covariances = msg->peopleSet[target_person].covariances;
        ROS_INFO("HrengagementAlgNode::people_tracker_callback: new targetId=%d", goal_person_.targetId);     
        
        try
        {
          std::string tf_prefix;
          public_node_handle_.param<std::string>("tf_prefix", tf_prefix, "");
          std::string source = msg->header.frame_id;
          std::string target(tf_prefix+"/base_link");
          std::string fixed(tf_prefix+"/odom");
          ROS_INFO("HrengagementAlgNode::people_tracker_callback: source=%s target=%s",source.c_str(),target.c_str());

//          ros::Time now = msg->header.stamp;
//           ros::Time now = ros::Time::now();
          ros::Time now = ros::Time(0);
          ROS_INFO("HrengagementAlgNode::people_tracker_callback:    now=%f",ros::Time::now().toSec());
          ROS_INFO("HrengagementAlgNode::people_tracker_callback: header=%f",msg->header.stamp.toSec());

          //waitForTransform(target_frame, source_frame, time, timeout, polling_sleep_duration);
          bool tf_exists = tf_listener_.waitForTransform(target, source, now, ros::Duration(1));
          ROS_INFO("HrengagementAlgNode::people_tracker_callback: tf_exists=%d",tf_exists);

          if(!tf_exists)
          {
            ROS_WARN("HrengagementAlgNode::TF transform from %s to %s could NOT be found!",source.c_str(),target.c_str());
          }
          else
          {
            geometry_msgs::PoseStamped person_pose_laser_frame, person_pose_base_link;

            person_pose_laser_frame.header           = msg->header;
            person_pose_laser_frame.pose.position.x  = goal_person_.x;
            person_pose_laser_frame.pose.position.y  = goal_person_.y;
            person_pose_laser_frame.pose.orientation = tf::createQuaternionMsgFromYaw(0.f);

//             ROS_INFO("person_pose_laser_frame:: frame_id=%s target.frame_id=%s",
//               person_pose_laser_frame.header.frame_id.c_str(),
//               target.c_str());

            //transformPose(target_frame, target_time, pose_in, fixed_frame, pose_out);
            tf_listener_.transformPose(target, now, person_pose_laser_frame, fixed, person_pose_base_link);

            ROS_INFO("HrengagementAlgNode::person_pose_laser_frame:: frame_id=%s\tpose=(%f, %f)",
              person_pose_laser_frame.header.frame_id.c_str(),
              person_pose_laser_frame.pose.position.x,
              person_pose_laser_frame.pose.position.y);

            ROS_INFO("HrengagementAlgNode::person_pose_base_link::   frame_id=%s\tpose=(%f, %f)",
              person_pose_base_link.header.frame_id.c_str(),
              person_pose_base_link.pose.position.x,
              person_pose_base_link.pose.position.y);

            track_people_ = false;
            
            geometry_msgs::Point target_pose = person_pose_base_link.pose.position;
            target_pose = alg_.substractRobotSafetyDistance(target_pose);
//             person_pose_base_link.pose.position = target_pose;

            MoveBaseMakeActionRequest(person_pose_base_link);
          }
        }
        catch(tf::TransformException e)
        {
          ROS_ERROR("TF Error: %s",e.what());
        }
      }

    }
  //unlock previously blocked shared variables 
  people_tracker_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */
void HrengagementAlgNode::HRIDone(const actionlib::SimpleClientGoalState& state, const tibi_dabo_msgs::sequenceResultConstPtr& result)
{
  ROS_INFO("HrengagementAlgNode::HRIDone: %s", state.toString().c_str());
  if(state.state_==actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("HrengagementAlgNode::HRIDone: Goal Achieved!");
  }
  else if(state.state_==actionlib::SimpleClientGoalState::ABORTED)
  {
    ROS_INFO("HrengagementAlgNode::HRIDone: Sequence aborted!");
  }
  else
  {
    ROS_INFO("HrengagementAlgNode::HRIDone: Unknown termination state!");
  }

  track_people_ = true;
}

void HrengagementAlgNode::HRIActive(void)
{
  ROS_DEBUG("HrengagementAlgNode::HRIActive: Goal just went active!");
}

void HrengagementAlgNode::HRIFeedback(const tibi_dabo_msgs::sequenceFeedbackConstPtr& feedback)
{
}
    
void HrengagementAlgNode::MoveBaseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result) 
{ 
  ROS_INFO("HrengagementAlgNode::MoveBaseDone: Goal Achieved!"); 
  ROS_INFO("HrengagementAlgNode::MoveBaseDone: %s", state.toString().c_str()); 

  //copy & work with requested result 
  
  HRIMakeActionRequest(alg_.loadHRISequence("greeting.xml"));
} 

void HrengagementAlgNode::MoveBaseActive(void) 
{ 
  ROS_INFO("HrengagementAlgNode::MoveBaseActive: Goal just went active!"); 
} 

void HrengagementAlgNode::MoveBaseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) 
{ 
  ROS_INFO("HrengagementAlgNode::MoveBaseFeedback: Got Feedback!"); 

  //analyze feedback 
  ROS_INFO("HrengagementAlgNode::MoveBaseFeedback: pose=(%f, %f)", feedback->base_position.pose.position.x, feedback->base_position.pose.position.y);

  //if feedback is not what expected, cancel requested goal 
//   if( !feedback_is_ok ) 
//   { 
//     MoveBase_client_.cancelGoal(); 
//     ROS_INFO("HrengagementAlgNode::MoveBaseFeedback: Cancelling Action!"); 
//   } 
}

/*  [action requests] */
void HrengagementAlgNode::HRIMakeActionRequest(const std::vector<std::string> & xml_files)
{
  ROS_DEBUG("HrengagementAlgNode::HRIMakeActionRequest: Starting New Request!");
  //wait for the action server to start 
  ROS_DEBUG("HrengagementAlgNode::HRIMakeActionRequest: Waiting for Server...");
  hri_client_.waitForServer();
  ROS_DEBUG("HrengagementAlgNode::HRIMakeActionRequest: Server is Available!");

  //set goal
  hri_goal_.sequence_file[TTS_]       = "Si no m'he equivocat estic davant d'una persona \\item=Laugh_01";//xml_files[TTS_];
  hri_goal_.sequence_file[LEDS_]      = "";//xml_files[LEDS_];
  hri_goal_.sequence_file[HEAD_]      = "head_greeting.xml";//xml_files[HEAD_];
  hri_goal_.sequence_file[RIGHT_ARM_] = "right_arm_greeting.xml";//xml_files[RIGHT_ARM_];
  hri_goal_.sequence_file[LEFT_ARM_]  = "left_arm_greeting.xml";//xml_files[LEFT_ARM_];

  //send a goal to the action 
  hri_client_.sendGoal(hri_goal_,
              boost::bind(&HrengagementAlgNode::HRIDone,     this, _1, _2),
              boost::bind(&HrengagementAlgNode::HRIActive,   this),
              boost::bind(&HrengagementAlgNode::HRIFeedback, this, _1));
}

void HrengagementAlgNode::MoveBaseMakeActionRequest(const geometry_msgs::PoseStamped & new_goal) 
{ 
  ROS_INFO("HrengagementAlgNode::MoveBaseMakeActionRequest: Starting New Request!"); 

  //wait for the action server to start 
  //will wait for infinite time 
  ROS_INFO("HrengagementAlgNode::MoveBaseMakeActionRequest: Waiting for Server..."); 
  MoveBase_client_.waitForServer();
  ROS_INFO("HrengagementAlgNode::MoveBaseMakeActionRequest: Server is Available!"); 

  //set goal
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header = new_goal.header;
  goal.target_pose.pose   = new_goal.pose;

  ROS_INFO("[%d] - NEW goal=(%f, %f, %f)", goal.target_pose.header.seq,
                                           goal.target_pose.pose.position.x,
                                           goal.target_pose.pose.position.y, 
                                           goal.target_pose.pose.position.z);
  
  //send a goal to the action 
  MoveBase_client_.sendGoal(goal, 
              boost::bind(&HrengagementAlgNode::MoveBaseDone,     this, _1, _2), 
              boost::bind(&HrengagementAlgNode::MoveBaseActive,   this), 
              boost::bind(&HrengagementAlgNode::MoveBaseFeedback, this, _1)); 
  ROS_INFO("HrengagementAlgNode::MoveBaseMakeActionRequest: Goal Sent. Wait for Result!"); 
  
  // wait for the action to return 
  float server_timeout = 100.f; //in [secs] 
  bool finished_before_timeout = MoveBase_client_.waitForResult(ros::Duration(server_timeout)); 

  //if server replies in time 
  if (finished_before_timeout) 
  { 
    actionlib::SimpleClientGoalState state = MoveBase_client_.getState(); 
    ROS_INFO("HrengagementAlgNode::MoveBaseMakeActionRequest: Action Succesfully Accomplished!"); 
    ROS_INFO("HrengagementAlgNode::MoveBaseMakeActionRequest: %s", state.toString().c_str()); 
  } 
  else 
  { 
    MoveBase_client_.cancelGoal();
    track_people_ = true; 
    ROS_INFO("HrengagementAlgNode::MoveBaseMakeActionRequest: Action did NOT finish before Timeout."); 
  } 
}

void HrengagementAlgNode::node_config_update(Config &config, uint32_t level)
{
}

void HrengagementAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<HrengagementAlgNode>(argc, argv, "hrengagement_alg_node");
}
