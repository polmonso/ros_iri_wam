#include "people_follower_client_alg_node.h"

PeopleFollowerClientAlgNode::PeopleFollowerClientAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<PeopleFollowerClientAlgorithm>(),
  follow_client_("followTarget", true),
  make_request_(true),
  target_id_(-1)
{
  //init class attributes if necessary
  loop_rate_ = 1;//in [Hz]

  // [init publishers]

  // [init subscribers]
  people_tracker_subscriber_ = public_node_handle_.subscribe("people_tracker", 100, &PeopleFollowerClientAlgNode::people_tracker_callback, this);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

PeopleFollowerClientAlgNode::~PeopleFollowerClientAlgNode(void)
{
  // [free dynamic memory]
}

void PeopleFollowerClientAlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]
  actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::StateEnum(1));
  iri_nav_msgs::followTargetResultConstPtr result;
//  std::cout << std::endl << "let's try this..." << std::endl;
//  followDone(state,result);
//  followDone2(state,result);
//  std::cout << "did it work?" << std::endl;

/*
  people_tracker_mutex_.enter();
    ROS_DEBUG("PeopleFollowerClientAlgNode::mainNodeThread:: make_request=%d", make_request_);
    if(make_request_)
      ROS_INFO("PeopleFollowerClientAlgNode::mainNodeThread: make_request=True");
  people_tracker_mutex_.exit();
*/
//   if(make_request_)
//   {
//     make_request_ = false;
//     followMakeActionRequest();
//     sleep(4);
//     followMakeActionRequest();
//   }

  // [publish messages]
}

/*  [subscriber callbacks] */
void PeopleFollowerClientAlgNode::people_tracker_callback(const iri_people_tracking::peopleTrackingArray::ConstPtr& msg) 
{

  //use appropiate mutex to shared variables if necessary
  people_tracker_mutex_.enter();
    ROS_DEBUG("PeopleFollowerClientAlgNode::people_tracker_callback: New Message Received (make_request=%d)",make_request_); 
    if(make_request_)
    {
      unsigned int target_index;
      if( !alg_.isSomeoneStanding(msg, target_index) )
      {
        ROS_WARN("PeopleFollowerClientAlgNode::No people to track...");
      }
      else
      {
        make_request_ = false;
        follow_goal_.target_id = msg->peopleSet[target_index].targetId;
        followMakeActionRequest();
      }
    }
  people_tracker_mutex_.exit();

}

/*  [service callbacks] */

/*  [action callbacks] */
void PeopleFollowerClientAlgNode::followDone(const actionlib::SimpleClientGoalState& state,  const iri_nav_msgs::followTargetResultConstPtr& result)
{
  if( state.toString().compare("SUCCEEDED") == 0 )
    ROS_WARN("PeopleFollowerClientAlgNode::followDone: Goal Achieved! %s", state.toString().c_str());
  else
    ROS_WARN("PeopleFollowerClientAlgNode::followDone: %s", state.toString().c_str());

  people_tracker_mutex_.enter();
    sleep(5);
    make_request_ = true;
    ROS_INFO("PeopleFollowerClientAlgNode::followDone: make_request=%d",make_request_);
  people_tracker_mutex_.exit();
}

void PeopleFollowerClientAlgNode::followActive()
{
  //ROS_INFO("PeopleFollowerClientAlgNode::followActive: Goal just went active!");
}

void PeopleFollowerClientAlgNode::followFeedback(const iri_nav_msgs::followTargetFeedbackConstPtr& feedback)
{
  //ROS_INFO("PeopleFollowerClientAlgNode::followFeedback: Got Feedback!");

  bool feedback_is_ok = true;

  //analyze feedback
  //my_var = feedback->var;

  //if feedback is not what expected, cancel requested goal
  if( !feedback_is_ok )
  {
    follow_client_.cancelGoal();
    //ROS_INFO("PeopleFollowerClientAlgNode::followFeedback: Cancelling Action!");
  }
}

/*  [action requests] */
void PeopleFollowerClientAlgNode::followMakeActionRequest()
{
  ROS_INFO("PeopleFollowerClientAlgNode::followMakeActionRequest: Starting New Request!");

  //wait for the action server to start
  //will wait for infinite time
  follow_client_.waitForServer();
  ROS_INFO("PeopleFollowerClientAlgNode::followMakeActionRequest: Server is Available!");

  //send a goal to the action
//   follow_goal_.target_id    = 1;
  follow_goal_.dist_to_goal = 0.8f;

  ROS_INFO("PeopleFollowerClientAlgNode::followMakeActionRequest: target_id=%d",follow_goal_.target_id);
  sleep(2);
  follow_client_.sendGoal(follow_goal_,
              boost::bind(&PeopleFollowerClientAlgNode::followDone,     this, _1, _2),
              boost::bind(&PeopleFollowerClientAlgNode::followActive,   this),
              boost::bind(&PeopleFollowerClientAlgNode::followFeedback, this, _1));
  ROS_INFO("PeopleFollowerClientAlgNode::followMakeActionRequest: Goal Sent. Wait for Result!");
}

void PeopleFollowerClientAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void PeopleFollowerClientAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<PeopleFollowerClientAlgNode>(argc, argv, "people_follower_client_alg_node");
}
