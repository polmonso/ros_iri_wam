#include "wam_move_arm_alg.h"
#include <iostream>
#include <fstream>
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
	if(T_total.toSec() > 0.0)
	{
	  double dTime = T_total.toSec() / current_trajectory.points.size();
	  for(unsigned int i=0; i < current_trajectory.points.size(); ++i)
	  {
		  current_trajectory.points[i].time_from_start = ros::Duration(i*dTime);
	  }	 
    }
    else 
    {
		std::vector<double> MaxVel = getMaxVelocities(current_trajectory.joint_names);
		getLongerTime(current_trajectory,MaxVel);
	}
}
void WamMoveArmAlgorithm::restoreVelocity(trajectory_msgs::JointTrajectory &current_trajectory,const ros::Duration& T_total)
{
  double q1,q0;
  double dTime = T_total.toSec() /current_trajectory.points.size();
   std::ofstream f2;
   f2.open("/home/irojas/Desktop/trajectorias.txt", std::ofstream::out);
  
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
	 f2<<current_trajectory.points[i].positions[0]<<" "<<current_trajectory.points[i].positions[1]<<" "<<current_trajectory.points[i].positions[2]<<" "<<current_trajectory.points[i].positions[3]<<" "<<current_trajectory.points[i].positions[4]<<" "<<current_trajectory.points[i].positions[5]<<" "<<current_trajectory.points[i].positions[6]<<"\r";
  }	 
  f2.close(); 
  
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
double WamMoveArmAlgorithm::getLongerTime(const std::vector<double> positions,const std::vector<double> maxVel)
{
	if(positions.size() != maxVel.size())
	{
		ROS_ERROR("[iri_wam_move_arm] Vector posiciones y vector de maxima velocidad con tamano distinto");
		exit(1);
	}
	double ax=0.0,tmp=0.0;
	for(size_t i=0; i < maxVel.size(); ++i)
	{
		tmp= positions[i] / maxVel[i];
		if((i == 0)||(ax < tmp))ax=tmp;
	}
	return ax;
}
void WamMoveArmAlgorithm::getLongerTime(trajectory_msgs::JointTrajectory &current_trajectory,const std::vector<double> maxVel)
{
	double time=0.0;
	for(size_t i=0; i < current_trajectory.points.size(); ++i)
	{
		time=getLongerTime(current_trajectory.points[i].positions,maxVel);
		current_trajectory.points[i].time_from_start =ros::Duration(time);
	}
}
std::vector<double> WamMoveArmAlgorithm::getMaxVelocities(std::vector<std::string> vecNames)
{
	std::vector<double> vec;
	vec.resize(7);
	ros::NodeHandle public_node_handle_("/");
	for(size_t i=0; i < vecNames.size(); ++i)
	{
	 public_node_handle_.param<double>("/trajectory_filter_server/joint_limits/"+vecNames[i]+"/max_velocity", vec[i], 1.0);
    }
	return vec;
}
