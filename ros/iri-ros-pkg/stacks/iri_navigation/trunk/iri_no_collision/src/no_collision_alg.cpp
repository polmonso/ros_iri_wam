#include "no_collision_alg.h"

#include<math.h>
#include<tf/tf.h>
#include<algorithm>

NoCollisionAlgorithm::NoCollisionAlgorithm(void) :
  MIN_SAFE_DISTANCE(0.8f),
  MIN_SAFE_ANGLE(20.f*M_PI/180.f),
  VT(0.2f),
  VR(0.1f),
  OA_CONE_ANGLE(100.0f)
{
  resetDistance2Goal();
}

NoCollisionAlgorithm::~NoCollisionAlgorithm(void)
{
}

void NoCollisionAlgorithm::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();
    // save the current configuration
    MIN_SAFE_DISTANCE = new_cfg.min_safefy_dist;
    MIN_SAFE_ANGLE    = new_cfg.min_safefy_angle*M_PI/180.f,
    VT = new_cfg.vT;
    VR = new_cfg.vR;
    OA_CONE_ANGLE = new_cfg.oa_cone_aperture;
    this->config_=new_cfg;
  this->unlock();
}

// NoCollisionAlgorithm Public API
void NoCollisionAlgorithm::resetDistance2Goal(void)
{
  this->lock();
    dist_to_goal_  = 2*MIN_SAFE_DISTANCE;
    angle_to_goal_ = 2*MIN_SAFE_ANGLE;
  this->unlock();
}

bool NoCollisionAlgorithm::isGoalReached()
{
  bool ret = false;
  this->lock();
    if( (dist_to_goal_ <= MIN_SAFE_DISTANCE) && (fabs(angle_to_goal_) <= MIN_SAFE_ANGLE) )
      ret = true;
  this->unlock();
  
  return ret;
}

geometry_msgs::Twist
NoCollisionAlgorithm::movePlatform(const sensor_msgs::LaserScan & scan, 
                                   const geometry_msgs::Pose & local_goal)
{
  geometry_msgs::Twist twist;
  twist.linear.x  = 0.f;
  twist.angular.z = 0.f;

  this->lock();
    float min_dist = MIN_SAFE_DISTANCE;

    if( isGoalTraversable(scan, min_dist, local_goal.position) )
    {
      ROS_INFO("NoCollisionAlgorithm::movePlatform::MOVE!");

      // compute angle between both vectors (robot turn angle)
      angle_to_goal_ = atan2(local_goal.position.y, local_goal.position.x);
      dist_to_goal_  = sqrt(local_goal.position.x*local_goal.position.x + local_goal.position.y*local_goal.position.y);

      // set vR
      if( fabs(angle_to_goal_) > MIN_SAFE_ANGLE )
      {
        twist.angular.z = VR * (fabs(angle_to_goal_)/angle_to_goal_);
      }
      else
      {
        // set vT
        if( dist_to_goal_ > MIN_SAFE_DISTANCE )
          twist.linear.x = VT;
      }
    }
    else
    {
      ROS_WARN("NoCollisionAlgorithm::movePlatform::STOP!");
      twist.linear.x  = 0.f;
      twist.angular.z = 0.f;
    }

  this->unlock();

  ROS_INFO("NoCollisionAlgorithm::movePlatform::twist:: vT=%f vR=%f", twist.linear.x, twist.angular.z);
  return twist;
}

void NoCollisionAlgorithm::fromCartesian2Polar(const geometry_msgs::Point & p, 
                                               float & module, 
                                               float & angle)
{
  module = sqrt( p.x*p.x + p.y*p.y );
  angle  = atan2( p.y, p.x );
}

bool NoCollisionAlgorithm::isGoalTraversable(const sensor_msgs::LaserScan & laser, 
                                             const float & safety_distance)
{
  unsigned int shields_offset = round(10*720.f/180.f);
  for(unsigned int ii=shields_offset; ii<laser.ranges.size()-shields_offset; ii++)
  {
    //TODO
    // check why laser is sending MIN_LASER_RANGE values
    if( laser.ranges[ii] > MIN_LASER_RANGE )
      if( laser.ranges[ii] <= safety_distance )
        return false;
  }
  
  return true;
}

bool NoCollisionAlgorithm::isGoalTraversable(const sensor_msgs::LaserScan & laser, 
                                             const float & safety_distance, 
                                             const geometry_msgs::Point & goal) const
{
  // convert goal from cartesian to polar coordinates
  float module_goal, angle_goal;
  fromCartesian2Polar(goal, module_goal, angle_goal);

  // index of the laser range corresponding to robot orientation
  const unsigned int central_index_range = round(laser.ranges.size()/2);

  // transform angle goal from robot coordinates to laser ranges
  const unsigned int goal_index_range = round(angle_goal/laser.angle_increment) + central_index_range;

  // total aperture angle in radians
  const double cone_aperture = OA_CONE_ANGLE*M_PI/180.f;
  const unsigned int half_index_cone = abs(round(cone_aperture/(laser.angle_increment*2)));

  // for all cone indexes inside laser scan
  for(unsigned int ii=std::max(goal_index_range-half_index_cone,(unsigned int)0);
                   ii<std::min(goal_index_range+half_index_cone,laser.ranges.size());
                   ii++)
  {
    // if there is an obstacle
    if ( laser.ranges[ii] > MIN_LASER_RANGE && laser.ranges[ii] <= safety_distance )
    {
      ROS_DEBUG("NoCollisionAlgorithm::isGoalTraversable::FALSE");
      return false;
    }
  }

  ROS_DEBUG("NoCollisionAlgorithm::isGoalTraversable::TRUE");
  return true;
}
