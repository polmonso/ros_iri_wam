#include "iri_wam_controllers/sim_trajectory_filter.h"

namespace sim_trajectory_filter
{
  sim_trajectory_filter::sim_trajectory_filter():
  private_hand_("~"),
  recieve_time(false),
   id_path(""),
   controller_name("")
  {
	 // public_hand_.param<std::string>("controller_action_name", controller_name, "/iri_ros_controller/joint_trajectory_action");
	  bringupSubcribers();
	  bringupActions();
	  
  }
  void sim_trajectory_filter::bringupActions()
  {
    controller_action_client_ = new JointExecutorActionClient("/iri_ros_controller/follow_joint_trajectory");
    if(!controller_action_client_) {
      ROS_ERROR("Controller action client hasn't been initialized yet");
      return ;
    }
    /*while(!controller_action_client_->waitForActionServerToStart(ros::Duration(1.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server to come up.");
      if(!public_hand_.ok()) {
        return ;
      }
    }*/
    ROS_INFO("Connected to the controller");
	
		std::string server="server";
	action_server_follow_.reset(new FJTAS(public_hand_,server,boost::bind(&sim_trajectory_filter::goalCBFollow, this, _1)));
	
  }
  void sim_trajectory_filter::bringupSubcribers()
  {
	  duration_path_sub_=public_hand_.subscribe("/sim_trajectory_server/duration_path", 1, &sim_trajectory_filter::durationCB, this);
	  ROS_INFO("Subcriber Started: ");
  }
  void sim_trajectory_filter::durationCB(const iri_wam_controllers::PathDuration::ConstPtr& msg)
  {
    time_for_path=msg->duration_path_expected;
    id_path=msg->header.frame_id;
    recieve_time=true;
  }
  void sim_trajectory_filter::goalCBFollow(GoalHandleFollow gh)
  {
	if(recieve_time)
	{
	  gh.setAccepted();
	  trajectory_msgs::JointTrajectory input = gh.getGoal()->trajectory;
	  trajectory_msgs::JointTrajectory output;
	  copyMsg(input,output);
	  double dt=0.0;
	  get_Dtime(output.points.size(),dt);
	  velocityToMsg(output,dt);
	  accelToMsg(output,dt);
	  timeToMsg(output,dt);
	  sendTrajectory(output);
	}
  }
  void sim_trajectory_filter::get_Dtime(const int& num_points,double& dtime)
  {
	  double tPath=time_for_path.toSec();
	  dtime= tPath/num_points;
  }
  void sim_trajectory_filter::calculateVelocity(const double& p2,const double& p1,const double& dt, double& vel)
  {
	  vel= (p2-p1)/dt;
  }
  void sim_trajectory_filter::calculateAccel(const double& v2,const double& v1,const double& dt, double& accel)
  {
	  accel= (v2-v1)/dt;
  }
  void sim_trajectory_filter::getPositionMsg(const trajectory_msgs::JointTrajectory& msg,const int& index,const int& indexJoint,double& pos)
  {
	  if (index <= 0) pos= 0;
	  else  pos=msg.points[index].positions[indexJoint];
  }
  void sim_trajectory_filter::getVelocityMsg(const trajectory_msgs::JointTrajectory& msg,const int& index,const int& indexJoint,double& vel)
  {
	  if (index <= 0) vel= 0;
	  else  vel =msg.points[index].velocities[indexJoint];
  }
  void sim_trajectory_filter::velocityToMsg(trajectory_msgs::JointTrajectory& msg,const double& dt)
  {
	  double pos1=0.0,pos2=0.0,vel=0.0;
	  for(unsigned int i=0; i < msg.points.size(); ++i)
	  {
		  for(unsigned int j=0; j < msg.points[i].positions.size(); ++j)
		  {
			  getPositionMsg(msg,i,j,pos2);
			  getPositionMsg(msg,i-1,j,pos1);	
			  calculateVelocity(pos2,pos1,dt,vel);		  
			  msg.points[i].velocities[j]=vel;
		  }
	  }
  }
  void sim_trajectory_filter::accelToMsg(trajectory_msgs::JointTrajectory& msg,const double& dt)
  {
	  double vel1=0.0,vel2=0.0,accel=0.0;
	  for(unsigned int i=0; i < msg.points.size(); ++i)
	  {
		  for(unsigned int j=0; j < msg.points[i].positions.size(); ++j)
		  {
			  getPositionMsg(msg,i,j,vel2);
			  getPositionMsg(msg,i-1,j,vel1);	
			  calculateAccel(vel2,vel1,dt,accel);		  
			  msg.points[i].velocities[j]=accel;
		  }
	  }	  
  }
  void sim_trajectory_filter::timeToMsg(trajectory_msgs::JointTrajectory& msg, const double& time)
  {
	  for(unsigned int i=0; i < msg.points.size(); ++i)
	  {
		  msg.points[i].time_from_start= ros::Duration(time*i);
	  }
  }
  void sim_trajectory_filter::sendGoal(const control_msgs::FollowJointTrajectoryGoal& msg)
  {
	  controller_action_client_->sendGoal(msg);
  }
  void sim_trajectory_filter::copyMsg(const trajectory_msgs::JointTrajectory& a,trajectory_msgs::JointTrajectory& b)
  {
	  b.header=a.header;	
	  b.joint_names=a.joint_names;	
	  b.points=a.points;
  }
  void sim_trajectory_filter::sendTrajectory(const trajectory_msgs::JointTrajectory& msg)
  {
    control_msgs::FollowJointTrajectoryGoal goal;  
    goal.trajectory = msg;
	 sendGoal(goal); 
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_trajectory_filter");
  sim_trajectory_filter::sim_trajectory_filter server;
  ROS_INFO("Simulated Trajectory Filter started");
  ros::spin(); 
  return 0;
}
