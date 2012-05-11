#include "wam_move_arm_alg.h"

WamMoveArmAlgorithm::WamMoveArmAlgorithm(void)
{
}

WamMoveArmAlgorithm::~WamMoveArmAlgorithm(void)
{
}

void WamMoveArmAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// WamMoveArmAlgorithm Public API
void WamMoveArmAlgorithm::setTime(const ros::Duration msg)
{
	path_time_= msg;
}
ros::Duration WamMoveArmAlgorithm::getTime()
{
	return path_time_;
}
void WamMoveArmAlgorithm::restoreTime(trajectory_msgs::JointTrajectory &current_trajectory,const ros::Duration& T_total)
{
  double dTime = T_total.toSec() / current_trajectory.points.size();
  for(unsigned int i=0; i < current_trajectory.points.size(); ++i)
  {
	  current_trajectory.points[i].time_from_start = ros::Duration(i*dTime);
  }	  
}
void WamMoveArmAlgorithm::restoreVelocity(trajectory_msgs::JointTrajectory &current_trajectory,const ros::Duration& T_total)
{
  double q1,q0;
  double dTime = T_total.toSec() /current_trajectory.points.size();
  for(unsigned int i=0; i < current_trajectory.points.size(); ++i)
  {
	 for(unsigned int j=0; j < current_trajectory.points[i].velocities.size(); ++j)
	 {
			if(i == 0)current_trajectory.points[i].velocities[j]= 0.0;
			else
			{		 
			  q1=current_trajectory.points[i].positions[j];
			  q0=current_trajectory.points[i-1].positions[j];
			  current_trajectory.points[i].velocities[j]= ((q1-q0)/dTime);		 
			}			 
	 }
  }	  
  
}
void WamMoveArmAlgorithm::restoreAccel(trajectory_msgs::JointTrajectory &current_trajectory,const ros::Duration& T_total)
{
  double q1,q0;
  double dTime = T_total.toSec() /current_trajectory.points.size();
  for(unsigned int i=0; i < current_trajectory.points.size(); ++i)
  {
	 for(unsigned int j=0; j < current_trajectory.points[i].accelerations.size(); ++j)
	 {
			if(i == 0)current_trajectory.points[i].accelerations[j]= 0.0;
			else
			{		 
			  q1=current_trajectory.points[i].velocities[j];
			  q0=current_trajectory.points[i-1].velocities[j];
			  current_trajectory.points[i].accelerations[j]= ((q1-q0)/dTime);		 
			}			 
	 }
  }	  	  
}
