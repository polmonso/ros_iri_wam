#include "hrengagement_alg.h"

HrengagementAlgorithm::HrengagementAlgorithm(void) :
  people_stand_max_vel_(0.1f),
  min_stopped_iters_(3)
{
}

HrengagementAlgorithm::~HrengagementAlgorithm()
{
}

void HrengagementAlgorithm::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  min_stopped_iters_    = new_cfg.min_stopped_iters;
  people_stand_max_vel_ = new_cfg.people_stand_max_vel;
  min_safefy_dist_      = new_cfg.min_safefy_dist;
  this->config_=new_cfg;
  
  this->unlock();
}

bool HrengagementAlgorithm::isSomeoneStanding(const iri_people_tracking::peopleTrackingArray::ConstPtr& msg, 
                                              unsigned int & target_index)
{
  bool ret = false;

  ROS_INFO("HrengagementAlgorithm::isSomeoneStanding: msg->peopleSet.size=%d", msg->peopleSet.size());

  this->lock();
  std::vector<TargetPeople> vCurrentTargets;

  //for all tracked persons
  for(unsigned int ii=0; ii<msg->peopleSet.size(); ii++)
  {
    unsigned int target_id = (unsigned int)abs(msg->peopleSet[ii].targetId);

    //compute current velocity
    float mod_vel = sqrt(msg->peopleSet[ii].vx*msg->peopleSet[ii].vx + 
                         msg->peopleSet[ii].vy*msg->peopleSet[ii].vy);

    //if current velocity is below threshold
    //person is stopped
    if( mod_vel < people_stand_max_vel_ )
    {
      //check if this person was already tracked
      bool found = false;
      unsigned int jj;
      for(jj=0; jj<vTargets_.size(); jj++)
      {
        if( target_id == vTargets_[jj].target_id )
        {
          ROS_INFO("HrengagementAlgorithm::isSomeoneStanding: target_id=%d FOUND in jj=%d!", target_id, jj);
          found = true;
          break;
        }
      }
    
      ROS_INFO("HrengagementAlgorithm::isSomeoneStanding: target_id=%d STANDING!", target_id);

      //if target was already tracked
      if(found)
        vCurrentTargets.push_back( TargetPeople(target_id, ++vTargets_[jj].stop_iter) );
      //if first time
      else
        vCurrentTargets.push_back( TargetPeople(target_id) );

      ROS_INFO("HrengagementAlgorithm::isSomeoneStanding: [%d][%d] target_id=%d stop_iter=%d", ii, jj, target_id, vCurrentTargets[jj].stop_iter);
    }
    //person is moving
    else
      ROS_INFO("HrengagementAlgorithm::isSomeoneStanding: targetId=%d MOVING", target_id);
  }

  //for all current tracked targets
  target_index = msg->peopleSet.size()+1;
  float min_dist = 1000000000.f;
  for(unsigned int ii=0; ii<vCurrentTargets.size(); ii++)
  {
    //if number of stopped iterations greater than threshold
    if( vCurrentTargets[ii].stop_iter > min_stopped_iters_)
    {
      float dist = sqrt(msg->peopleSet[ii].x*msg->peopleSet[ii].x + msg->peopleSet[ii].y*msg->peopleSet[ii].y);
      ROS_INFO("HrengagementAlgorithm::isSomeoneStanding: \ttargetId=%d pose=(%f, %f) dist=%f", 
               msg->peopleSet[ii].targetId, msg->peopleSet[ii].x, msg->peopleSet[ii].y, dist);
      
      //check if it is the current minimum distance
      if( dist < min_dist )
      {
        target_index = ii;
        min_dist     = dist;
      }
    }
    else
      ROS_INFO("HrengagementAlgorithm::isSomeoneStanding: \ttargetId=%d NOT stopped for enough time: %d", msg->peopleSet[ii].targetId, vCurrentTargets[ii].stop_iter);
  }

  //save current targets
  vTargets_ = vCurrentTargets;
  if(target_index != msg->peopleSet.size()+1)
  {
    ROS_INFO("HrengagementAlgorithm::isSomeoneStanding: Final Candidate: target_id=%d", msg->peopleSet[target_index].targetId);
    ret = true;
  }
  
  this->unlock();

  return ret;
}

geometry_msgs::Point 
HrengagementAlgorithm::substractRobotSafetyDistance(const geometry_msgs::Point & goal_pose)
{
  geometry_msgs::Point robot_pose;
  
  this->lock();
    //compute safety module by substracting safety distance
    float safety_mod = sqrt(goal_pose.x*goal_pose.x + goal_pose.y*goal_pose.y) - min_safefy_dist_;
  this->unlock();
  
  //recompute coordinates
  robot_pose.x = safety_mod*goal_pose.x;
  robot_pose.y = safety_mod*goal_pose.y;

  ROS_INFO("from goal_pose(%f,%f) to robot_pose(%f,%f)",goal_pose.x,goal_pose.y,robot_pose.x,robot_pose.y);
  return robot_pose;
}

std::vector<std::string> 
HrengagementAlgorithm::loadHRISequence(const std::string & filename)
{
  std::vector<std::string> xml_files;
  
//   try
//   {
//     std::auto_ptr<hri_config_t> cfg(hri_config(filename,xml_schema::flags::dont_validate));
//  
//     xml_files.resize(NUM_CLIENTS);
//     xml_files[LEDS_]=cfg->face_expression();
//     xml_files[HEAD_]=cfg->head_seq();
//     xml_files[LEFT_ARM_]=cfg->left_arm_seq();
//     xml_files[RIGHT_ARM_]=cfg->right_arm_seq();
//     xml_files[TTS_]=cfg->tts_text(); 
//   }
//   catch(const xml_schema::exception& e)
//   {
//     std::ostringstream os;
//     os << e;
//     /* handle exceptions */
//     std::cout << os.str() << std::endl;
//     throw;
//   }

  return xml_files;
}

// HrengagementAlgorithm Public API
//hre_state HrengagementAlgorithm::hre_update(const float & dist, const float & angle)
//{
//  return hre_.hre_update(dist, angle);
//}
