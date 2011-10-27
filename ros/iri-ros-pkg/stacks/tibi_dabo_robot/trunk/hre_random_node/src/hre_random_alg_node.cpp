#include "hre_random_alg_node.h"

static unsigned int hri_max_wait = 5000; //[ms]
//static unsigned int motion_max_wait = 20000; //[ms]
//static unsigned int move_action_timeout = 30000; // [ms]

HreRandomAlgNode::HreRandomAlgNode(void) :
  hri_client_("hri", true),
  MoveBase_client_("MoveBase", true)
{
  //init class attributes if necessary
  this->loop_rate_ = 100;//in [Hz]

  if(chdir(node_path.c_str())!=0)
    std::cout << "error changing the directory" << std::endl;


  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]

  hri_goal_.sequence_file   = std::vector<std::string>(NUM_CLIENTS, std::string(""));
  hri_goal_.num_repetitions = std::vector<int>(NUM_CLIENTS, 0);
 
  this->sequence_file="";
  this->operation_mode=0;
  this->hri_action_in_progress=false;
  this->move_action_in_progress=false;
}

HreRandomAlgNode::~HreRandomAlgNode(void)
{
  this->hre_access.enter();
  std::cout << "destructor hre" << std::endl;
  // [free dynamic memory]
  this->hre_access.exit();
}

void HreRandomAlgNode::mainNodeThread(void)
{
  std::vector<std::string> xml_files;
  static int hri_time_count=0;
//  static int motion_time_count=(rand()%motion_max_wait);//in [ms]
//  static int move_timeout=move_action_timeout;
//  float x,y,theta;

  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->hre_access.enter();
  if(this->operation_mode==0)// random mode
  {
    if(!this->hri_action_in_progress)
    {
      if(hri_time_count!=0)
      {
         hri_time_count-=10;
         if(hri_time_count<0) 
           hri_time_count=0;
      }
      else
      {
        try{
          this->alg_.load_random_goal(xml_files);
          this->HRIMakeActionRequest(xml_files);
          hri_time_count=(rand()%hri_max_wait);
        }catch(...){
          /* do nothing but avoid crashing */
          std::cout << "exception" << std::endl;
        }
      }
    }
//    if(!this->move_action_in_progress)
//    {
//      if(motion_time_count!=0)
//      {
//         motion_time_count-=10;
//         if(motion_time_count<0) 
//           motion_time_count=0;
//      }
//      else
//      {
//        this->alg_.load_random_motion(&x,&y,&theta);
//        std::cout << "---------------------------------move to: (" << x << "," << y << ") with orientation " << theta << std::endl;
//        this->MoveBaseMakeActionRequest(x,y,theta);
//        motion_time_count=(rand()%motion_max_wait)+3000;
//        move_timeout=move_action_timeout;
//      }
//    }
//    else
//    {
//      move_timeout-=10;
//      if(move_timeout==0)
//      {
//        ROS_INFO("TIMEOUT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
//        this->MoveBase_client_.cancelGoal();
//        this->move_action_in_progress=false;
//      }
//    }
  }
  else// sequence mode
  {
  }

  this->hre_access.exit();
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */
void HreRandomAlgNode::HRIDone(const actionlib::SimpleClientGoalState& state, const tibi_dabo_msgs::sequenceResultConstPtr& result)
{
  ROS_INFO("HreRandomAlgNode::HRIDone: %s", state.toString().c_str());
  if(state.state_==actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("HreRandomAlgNode::HRIDone: Goal Achieved!");
  }
  else if(state.state_==actionlib::SimpleClientGoalState::ABORTED)
  {
    ROS_INFO("HreRandomAlgNode::HRIDone: Sequence aborted!");
  }
  else
  {
    ROS_INFO("HreRandomAlgNode::HRIDone: Unknown termination state!");
  }
  this->hri_action_in_progress=false;
}

void HreRandomAlgNode::HRIActive(void)
{
  ROS_DEBUG("HreRandomAlgNode::HRIActive: Goal just went active!");
}

void HreRandomAlgNode::HRIFeedback(const tibi_dabo_msgs::sequenceFeedbackConstPtr& feedback)
{
//  ROS_INFO("HreRandomAlgNode::HRIFeedback: Got Feedback!");

//  ROS_INFO("HreRandomAlgNode:: %f of leds sequence completed", feedback->percentage[LEDS_CLIENT]);
//  ROS_INFO("HreRandomAlgNode:: %f of head sequence completed", feedback->percentage[HEAD_CLIENT]);
//  ROS_INFO("HreRandomAlgNode:: %f of left arm sequence completed", feedback->percentage[LEFT_ARM_CLIENT]);
//  ROS_INFO("HreRandomAlgNode:: %f of right arm sequence completed", feedback->percentage[RIGHT_ARM_CLIENT]);
}

void HreRandomAlgNode::MoveBaseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO("HreRandomAlgNode::MoveBaseDone: Goal Achieved!");
  ROS_INFO("HreRandomAlgNode::MoveBaseDone: %s", state.toString().c_str());

  //copy & work with requested result 
  this->move_action_in_progress=false;
}

void HreRandomAlgNode::MoveBaseActive(void)
{
  ROS_INFO("HreRandomAlgNode::MoveBaseActive: Goal just went active!");
}

void HreRandomAlgNode::MoveBaseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
  ROS_DEBUG("HreRandomAlgNode::MoveBaseFeedback: Got Feedback!");

  //analyze feedback 
  ROS_DEBUG("HreRandomAlgNode::MoveBaseFeedback: pose=(%f, %f)", feedback->base_position.pose.position.x, feedback->base_position.pose.position.y);

}

/*  [action requests] */
void HreRandomAlgNode::HRIMakeActionRequest(std::vector<std::string> &xml_files)
{
  ROS_DEBUG("HreRandomAlgNode::HRIMakeActionRequest: Starting New Request!");
  //wait for the action server to start 
  ROS_DEBUG("HreRandomAlgNode::HRIMakeActionRequest: Waiting for Server...");
  hri_client_.waitForServer();
  ROS_DEBUG("HreRandomAlgNode::HRIMakeActionRequest: Server is Available!");

  //set goal
  hri_goal_.sequence_file[TTS_] = xml_files[TTS_];
  hri_goal_.sequence_file[LEDS_] = xml_files[LEDS_];
  hri_goal_.sequence_file[HEAD_] = xml_files[HEAD_];
  hri_goal_.sequence_file[RIGHT_ARM_] = xml_files[RIGHT_ARM_];
  hri_goal_.sequence_file[LEFT_ARM_] = xml_files[LEFT_ARM_];

  //send a goal to the action 
  hri_client_.sendGoal(hri_goal_,
              boost::bind(&HreRandomAlgNode::HRIDone,     this, _1, _2),
              boost::bind(&HreRandomAlgNode::HRIActive,   this),
              boost::bind(&HreRandomAlgNode::HRIFeedback, this, _1));
  this->hri_action_in_progress=true;
}

void HreRandomAlgNode::MoveBaseMakeActionRequest(float x,float y,float theta)
{
  static int request_=0;

  ROS_INFO("HreRandomAlgNode::MoveBaseMakeActionRequest: Starting New Request!");

  //wait for the action server to start 
  //will wait for infinite time 
  ROS_INFO("HreRandomAlgNode::MoveBaseMakeActionRequest: Waiting for Server...");
  MoveBase_client_.waitForServer();
  ROS_INFO("HreRandomAlgNode::MoveBaseMakeActionRequest: Server is Available!");

  //set goal
  MoveBase_goal_.target_pose.header.seq         = ++request_;
  MoveBase_goal_.target_pose.header.stamp       = ros::Time::now();
  MoveBase_goal_.target_pose.header.frame_id    = "odom";
  MoveBase_goal_.target_pose.pose.position.x    = x;
  MoveBase_goal_.target_pose.pose.position.y    = y;
  MoveBase_goal_.target_pose.pose.position.z    = 0.f;
  MoveBase_goal_.target_pose.pose.orientation.x = 0.f;
  MoveBase_goal_.target_pose.pose.orientation.y = 0.f;
  MoveBase_goal_.target_pose.pose.orientation.z = theta;
  MoveBase_goal_.target_pose.pose.orientation.w = 0.f;

  //send a goal to the action 
  MoveBase_client_.sendGoal(MoveBase_goal_,
              boost::bind(&HreRandomAlgNode::MoveBaseDone,     this, _1, _2),
              boost::bind(&HreRandomAlgNode::MoveBaseActive,   this),
              boost::bind(&HreRandomAlgNode::MoveBaseFeedback, this, _1));
  ROS_INFO("HreRandomAlgNode::MoveBaseMakeActionRequest: Goal Sent. Wait for Result!");
  this->move_action_in_progress=true;
}


void HreRandomAlgNode::node_config_update(Config &config, uint32_t level)
{
  std::vector<std::string> xml_files;

  this->hre_access.enter();
//  if(this->operation_mode!=config.op_mode)
//  {
    /* change the operation mode */
//    this->operation_mode=config.op_mode;
    /* stop the platform if it is moving*/
//    if(this->move_action_in_progress)
//    {
//      this->MoveBase_client_.cancelGoal();
//      this->move_action_in_progress=false;
//    }
    /* stop the current hri sequence if any */
//    if(hri_action_in_progress)
//    {
//      this->hri_client_.cancelGoal();
//      this->hri_action_in_progress=false;
//    }
//    if(this->operation_mode==1)
//    {
      /* start the sequence */
//      try{
//        this->sequence_file=motion_path+config.seq_file;
//        this->alg_.load_goal(this->sequence_file,xml_files);
//        this->HRIMakeActionRequest(xml_files);
//      }catch(...){
        /* do nothing but avoid crashing */
//      }
//    }
//  } 
//  else
//  {
//    if(config.op_mode==1)
//    {
      /* load the new sequence and start it */
//      this->sequence_file=motion_path+config.seq_file;
      /* stop the platform if it is moving*/
//      if(this->move_action_in_progress)
//      {
//        this->MoveBase_client_.cancelGoal();
//        this->move_action_in_progress=false;
//      }
      /* stop the current hri sequence if any */
//      if(this->hri_action_in_progress)
//      {
//        this->hri_client_.cancelGoal();
//        this->hri_action_in_progress=false;
//      }
      /* start the sequence */
//      try{
//        this->alg_.load_goal(this->sequence_file,xml_files);
//        this->HRIMakeActionRequest(xml_files);
//      }catch(...){
        /* do nothing but avoid crashing */
//      }
//    }
//  }
  this->hre_access.exit(); 
}

void HreRandomAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<HreRandomAlgNode>(argc, argv, "hre_random_alg_node");
}
