#include <algorithm>
#include <assert.h>
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <gazebo/Body.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/Pose3d.hh>
#include <gazebo/SetModelConfiguration.h>
#include <gazebo/Simulator.hh>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include "wam_gazebo.h"
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <math.h>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("wam_gazebo", WAMGazebo);
typedef  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> ExecutorActionServer;
typedef  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle GoalHandle;	
typedef  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

typedef  control_msgs::FollowJointTrajectoryActionFeedback feedback;
enum{ JOINT1 , JOINT2 , JOINT3 , JOINT4 , JOINT5 , JOINT6 , JOINT7};
enum{ BODY1  , BODY2  , BODY3  , BODY4  , BODY5  , BODY6 , BODY7, BODY8};
static const std::string Name_Service="/gazebo/GeneralState";
WAMGazebo::WAMGazebo(Entity * parent) :
    Controller(parent),
    server(nodo_, "iri_wam_controller/follow_joint_trajectory", false),
    alive_(true),
    inMoving(false)
   {
  parent_ = dynamic_cast<Model *> (parent);
 
  if (!parent_)
    gzthrow("Differential_Position2d controller requires a Model as its parent");
  ROS_INFO("[wam_gazebo] Constructor");
  Param::Begin(&parameters);
  robotNamespaceP = new ParamT<std::string> ("robotNamespace", "/", 0);
  body_1          = new ParamT<std::string> ("b1","", 1);
  body_2          = new ParamT<std::string> ("b2","", 1);
  body_3          = new ParamT<std::string> ("b3","", 1);
  body_4          = new ParamT<std::string> ("b4","", 1);
  body_5          = new ParamT<std::string> ("b5","", 1);
  body_6          = new ParamT<std::string> ("b6","", 1);
  body_7          = new ParamT<std::string> ("b7","", 1);
  body_8          = new ParamT<std::string> ("b8","", 1);
  joint_1         = new ParamT<std::string> ("joint1","", 1);
  joint_2         = new ParamT<std::string> ("joint2","", 1);
  joint_3         = new ParamT<std::string> ("joint3","", 1);
  joint_4         = new ParamT<std::string> ("joint4","", 1);
  joint_5         = new ParamT<std::string> ("joint5","", 1);
  joint_6         = new ParamT<std::string> ("joint6","", 1);
  joint_7         = new ParamT<std::string> ("joint7","", 1);
  Param::End();
  server.registerGoalCallback(boost::bind(&WAMGazebo::goalCB, this, _1));
  server.start();
}
WAMGazebo::~WAMGazebo()
{
  delete robotNamespaceP;
  delete body_1;
  delete body_2;
  delete body_3;
  delete body_4;
  delete body_5;
  delete body_6;
  delete body_7;
  delete body_8;
  delete joint_1;
  delete joint_2;
  delete joint_3;
  delete joint_4;
  delete joint_5;
  delete joint_6;
  delete joint_7;	
}
// Initialize the controller
void WAMGazebo::InitChild()
{
  ROS_INFO("init child");
  callback_queue_thread_ = new boost::thread(boost::bind(&WAMGazebo::QueueThread, this));
}

// Load the controller
void WAMGazebo::LoadChild(XMLConfigNode *node)
{  
  ROS_INFO("Load child");
  robotNamespaceP->Load(node);
  robotNamespace = robotNamespaceP->GetValue();
  
  body_1 ->Load(node);
  body_2 ->Load(node);
  body_3 ->Load(node);
  body_4 ->Load(node);
  body_5 ->Load(node);
  body_6 ->Load(node);
  body_7 ->Load(node);  
  body_8 ->Load(node);  
  
  names_bodys.push_back(body_8->GetAsString()); 
  
  joint_1->Load(node);
  joint_2->Load(node);
  joint_3->Load(node);
  joint_4->Load(node);
  joint_5->Load(node);
  joint_6->Load(node);
  joint_7->Load(node); 

  names_joints.push_back(joint_1->GetAsString());
  names_joints.push_back(joint_2->GetAsString());
  names_joints.push_back(joint_3->GetAsString());
  names_joints.push_back(joint_4->GetAsString());  
  names_joints.push_back(joint_5->GetAsString());
  names_joints.push_back(joint_6->GetAsString());
  names_joints.push_back(joint_7->GetAsString()); 

  bodys_.push_back(parent_->GetBody(** body_1));
  bodys_.push_back(parent_->GetBody(** body_2));
  bodys_.push_back(parent_->GetBody(** body_3));
  bodys_.push_back(parent_->GetBody(** body_4));
  bodys_.push_back(parent_->GetBody(** body_5));
  bodys_.push_back(parent_->GetBody(** body_6));
  bodys_.push_back(parent_->GetBody(** body_7));  
  bodys_.push_back(parent_->GetBody(** body_8));  
  bodys_[BODY1]->SetStatic(true);
/*  bodys_[BODY2]->SetStatic(true);
  bodys_[BODY3]->SetStatic(true);
  bodys_[BODY4]->SetStatic(true);
  bodys_[BODY5]->SetStatic(true);
  bodys_[BODY6]->SetStatic(true);
  bodys_[BODY7]->SetStatic(true);
  bodys_[BODY8]->SetStatic(true);*/
  zeroBase=bodys_[0]->GetWorldPose();

//bodys_[0]->GetAnchor();
  joints_.push_back(parent_->GetJoint(** joint_1));
  joints_.push_back(parent_->GetJoint(** joint_2));
  joints_.push_back(parent_->GetJoint(** joint_3));
  joints_.push_back(parent_->GetJoint(** joint_4));  
  joints_.push_back(parent_->GetJoint(** joint_5));
  joints_.push_back(parent_->GetJoint(** joint_6));
  joints_.push_back(parent_->GetJoint(** joint_7));   
  joints_positions.resize(7);
  //anchors.resize(7);
  anchors.push_back(joints_[JOINT1]->GetAnchor(2));
  anchors.push_back(joints_[JOINT2]->GetAnchor(1));
  anchors.push_back(joints_[JOINT3]->GetAnchor(2));
  anchors.push_back(joints_[JOINT4]->GetAnchor(2));
  anchors.push_back(joints_[JOINT5]->GetAnchor(2));
  anchors.push_back(joints_[JOINT6]->GetAnchor(2));
  anchors.push_back(joints_[JOINT7]->GetAnchor(2));
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "wam_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(robotNamespace);
  initTopics(); 
  getPoses(init_positions);
  copyVector(init_positions,actual_positions);
  getState(joints_positions);
  traj_client_=new TrajClient("arm_controller",true);
  }
// Reset
void WAMGazebo::ResetChild()
{
  ROS_INFO("reset child");
  loadPose(init_positions);
  getState(joints_positions);
  copyVector(init_positions,actual_positions);
 }
void WAMGazebo::UpdateChild()
{ //lock.locked();
  loadPose(actual_positions);
  sendState(joints_positions);
  publishFeed(0);
  //ROS_INFO("UPDATE");
  //ros::Duration(0.0005).sleep();
}
void WAMGazebo::FiniChild()
{
  alive_=false;
  queue_.clear();
  queue_.disable();
  callback_queue_thread_->join();	
  rosnode_->shutdown();
}
void WAMGazebo::QueueThread()                                                                                                              
{
  static const double timeout = 0.05;
  ROS_INFO("[wam_GAZEBO] In the Queue Thread");
  while (alive_ && rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}
void WAMGazebo::goalCB(GoalHandle gh)
{
 lock.lock();  
 gh.setAccepted();
 control_msgs::FollowJointTrajectoryGoal::ConstPtr gl = gh.getGoal();
 /*trajectory_msgs::JointTrajectoryPoint jtp;
 getState(joints_positions);
  gh.setAccepted();
 for(unsigned int i=0; i< gl->trajectory.points.size(); ++i)
 {
	  
   jtp=gl->trajectory.points[i];	
   ros::Duration(0.05).sleep();
   pointToMove(jtp,gl->trajectory.joint_names);
   getState(joints_positions);
   sendState(joints_positions);
 }*/
   
	// while(!traj_client_->waitForActionServerToStart(ros::Duration(1.0)))
	 while(!traj_client_->waitForServer(ros::Duration(1.0)))
  	 {
       ROS_INFO("Waiting for the joint_trajectory_action server");
     }
	pr2_controllers_msgs::JointTrajectoryGoal goal;
    goal.trajectory=gl->trajectory;
 	//goal.trajectory.points.resize(1);
	//goal.trajectory.points[0]=jtp;
    traj_client_->sendGoal(goal);
    /*while(!traj_client_->getState().isDone() && ros::ok())
    {
     usleep(50000);
    }*/
 // command.publish(gl->trajectory);
  gh.setSucceeded();
  publishFeed(3);
  lock.unlock();
}

void WAMGazebo::copyVector(const std::vector<Pose3d>& a,std::vector<Pose3d>& b)
{
	if(b.size() > 0) b.clear();
	for(unsigned int ii=0; ii < a.size(); ++ii){
		b.push_back(a[ii]);
	}
}
void WAMGazebo::loadPose(const std::vector<Pose3d>& pose)
{
	bodys_[BODY1]->SetWorldPose(zeroBase,true);
	bodys_[BODY2]->SetWorldPose(pose[1],true);
	bodys_[BODY3]->SetWorldPose(pose[2],true);
	bodys_[BODY4]->SetWorldPose(pose[3],true);
	bodys_[BODY5]->SetWorldPose(pose[4],true);
    bodys_[BODY6]->SetWorldPose(pose[5],true);
	bodys_[BODY7]->SetWorldPose(pose[6],true);
	bodys_[BODY8]->SetWorldPose(pose[7],true);
	/*bodys_[BODY2]->SetRelativePose(pose[1],true);
	bodys_[BODY3]->SetRelativePose(pose[2],true);
	bodys_[BODY4]->SetRelativePose(pose[3],true);
	bodys_[BODY5]->SetRelativePose(pose[4],true);
    bodys_[BODY6]->SetRelativePose(pose[5],true);
	bodys_[BODY7]->SetRelativePose(pose[6],true);
	bodys_[BODY8]->SetRelativePose(pose[7],true);*/
}
void WAMGazebo::initTopics()
{
  state_publisher=rosnode_->advertise< sensor_msgs::JointState>("joint_states", 1);		
  service_=  global_.advertiseService(Name_Service,&WAMGazebo::GenerealStateCB,this);
  
  
 //ros::SubscribeOptions so =ros::SubscribeOptions::create<trajectory_msgs::JointTrajectory>("command", 1,
  	//															boost::bind(&WAMGazebo::commandCB, this, _1),ros::VoidPtr(), &queue_);  																
  //sub_command= rosnode_->subscribe(so);
  //command=rosnode_->advertise<trajectory_msgs::JointTrajectory>("command", 5);		 
  init_positions.resize(8);
  actual_positions.resize(8);
 }
void WAMGazebo::getState(std::vector<double>& jnt)
{
	if(jnt.size() > 0)jnt.clear();
	double pos=0;
	for(int i=JOINT1; i<=JOINT7;++i)
	{		
		getPosition(joints_[i],pos);
		jnt.push_back(pos);
	}
}
void WAMGazebo::sendState(const std::vector<double>& jnt)
{
 if(jnt.size() == 7)
 {
  sensor_msgs::JointState joint_msgs; 
  joint_msgs.name.resize(names_joints.size());
  joint_msgs.name=names_joints; 
  joint_msgs.position.resize(7);
  joint_msgs.position=jnt;
  joint_msgs.header.frame_id="wambase";
  joint_msgs.header.stamp=ros::Time::now();
  if(joint_msgs.position.size() == 7 && joint_msgs.name.size()==7)state_publisher.publish(joint_msgs);
 }
}
void WAMGazebo::pointToMove(trajectory_msgs::JointTrajectoryPoint& jtp,std::vector<std::string> names)
{	   
   int count = parent_->GetJointCount();
   double target_pos=0.0,current_pos=0.0,Dangle=0.0,acel=0.0,vel=0.0,force=0.0;
   Body *parent=NULL,*child=NULL;
   for(int ii=0; ii < count; ++ii)
   {
	   findJointPosition(target_pos,names_joints[ii],names,jtp.positions);
	   findJointVelocity(vel,names_joints[ii],names,jtp.velocities);
	   findJointAccel(acel,names_joints[ii],names,jtp.accelerations);
	   Vector3 anchor=anchors[ii];
	   Vector3 axis = joints_[ii]->GetAxis(0);
	   joints_[ii]->SetVelocity(2,vel);
	   current_pos=joints_positions[ii];
	   parent= getParentBody(joints_[ii]);
	   child= getChildBody(joints_[ii]);
	   force=acel*(child->GetMass().GetAsDouble());
	   (ii != 1)?joints_[ii]->SetForce(2,force):joints_[ii]->SetForce(1,force);
	   Dangle= (target_pos-current_pos)*10000;
	   Dangle=round(Dangle);
	   Dangle/=10000;
	   rotateBodyAndChildren(child,anchor,axis,Dangle,true); 
   }
 }
bool WAMGazebo::findJointPosition(double &position, std::string name, std::vector<std::string> joint_names, std::vector<double> joint_positions)
{
  position = 0;
  std::vector<std::string>::iterator jit = joint_names.begin();
  std::vector<double>::iterator pit = joint_positions.begin();
  for(;jit != joint_names.end(),pit != joint_positions.end(); jit++,pit++)
  {
    if (name == (*jit))
    {
      position = (*pit);
      return true;
    }
  }
  return false;
}
bool WAMGazebo::findJointVelocity(double &velocity, std::string name, std::vector<std::string> joint_names, std::vector<double> joint_velocities)
{
  velocity = 0;
  std::vector<std::string>::iterator jit = joint_names.begin();
  std::vector<double>::iterator pit = joint_velocities.begin();
  for(;jit != joint_names.end(),pit != joint_velocities.end(); jit++,pit++)
  {
    if (name == (*jit))
    {
      velocity = (*pit);
      return true;
    }
  }
  return false;
}
bool WAMGazebo::findJointAccel(double &accel, std::string name, std::vector<std::string> joint_names, std::vector<double> joint_accelarations)
{
  accel = 0;
  std::vector<std::string>::iterator jit = joint_names.begin();
  std::vector<double>::iterator pit = joint_accelarations.begin();
  for(;jit != joint_names.end(),pit != joint_accelarations.end(); jit++,pit++)
  {
    if (name == (*jit))
    {
      accel = (*pit);
      return true;
    }
  }
  return false;
}
Body* WAMGazebo::getParentBody(Joint* joint)
{
  if (joint->GetJointBody(0) == joint->anchorBody)
    return joint->GetJointBody(1);
  else if (joint->GetJointBody(1) == joint->anchorBody)
    return joint->GetJointBody(0);
  else
    return NULL;
}
Body* WAMGazebo::getChildBody(Joint* joint)
{
  if (joint->GetJointBody(0) == joint->anchorBody)
     return joint->GetJointBody(0);
  else if (joint->GetJointBody(1) == joint->anchorBody)
     return joint->GetJointBody(1);
  else
     return NULL;
}    
void WAMGazebo::rotateBodyAndChildren(Body* body1,Vector3 anchor,Vector3 axis, double dangle, bool update_children)
{ 
  gazebo::Simulator::Instance()->GetMRMutex()->lock();  
  // Pose3d world_pose = body1->GetWorldPose();
  Pose3d world_pose = body1->GetRelativePose();
  Pose3d relative_pose(world_pose.pos - anchor,world_pose.rot); 
  Quatern rotation;
  rotation.SetFromAxis(axis.x,axis.y,axis.z,dangle);
  Pose3d new_relative_pose;
  new_relative_pose.pos = rotation.RotateVector(relative_pose.pos);
  new_relative_pose.rot = rotation * relative_pose.rot;
  Pose3d new_world_pose(relative_pose.pos+anchor,new_relative_pose.rot);
  body1->SetRelativePose(new_world_pose);
    //body1->SetWorldPose(new_world_pose);
  std::vector<Body*> bodies;
  if (update_children) getAllChildrenBodies(bodies, body1->GetModel(), body1);
  for (std::vector<Body*>::iterator bit = bodies.begin(); bit != bodies.end(); bit++)
    rotateBodyAndChildren((*bit), anchor, axis, dangle,false);
  if(update_children)
  {
	getPoses(actual_positions);   
  }
  gazebo::Simulator::Instance()->GetMRMutex()->unlock();  
}
void WAMGazebo::getAllChildrenBodies(std::vector<Body*> &bodies,Model* model, Body* body)
{
      // strategy, for each child, recursively look for children
      //           for each child, also look for parents to catch multiple roots
      if (model)
      {
        int joint_count = model->GetJointCount();
        for (int i = 0; i < joint_count ; i++)
        {
          gazebo::Joint* joint = model->GetJoint(i);
          // recurse through children connected by joints
          Body* body0 = getParentBody(joint);
          Body* body1 = getChildBody(joint);
          if (body0 && body1
              && body0->GetName() != body1->GetName()
              && body0->GetName() == body->GetName()
              && !inBodies(body1,bodies))
          {
            bodies.push_back(body1);
            getAllChildrenBodies(bodies, body1->GetModel(), body1);
            getAllParentBodies(bodies, body1->GetModel(), body1, body);
          }
        }
      }
    }
void WAMGazebo::getAllParentBodies(std::vector<Body*> &bodies,Model* model,Body* body,Body* orig_parent_body)
{
      if (model)
      {
        int joint_count = model->GetJointCount();
        for (int i = 0; i < joint_count ; i++)
        {
          Joint* joint = model->GetJoint(i);
          // recurse through children connected by joints
          Body* body0 = getParentBody(joint);
          Body* body1 = getChildBody(joint);
          if (body0 && body1
              && body0->GetName() != body1->GetName()
              && body1->GetName() == body->GetName()
              && body0->GetName() != orig_parent_body->GetName()
              && !inBodies(body0,bodies))
          {
            bodies.push_back(body0);
            getAllParentBodies(bodies, body0->GetModel(), body1, orig_parent_body);
          }
        }
      }
    }
bool WAMGazebo::inBodies(Body* body,std::vector<Body*> bodies)
{
      for (std::vector<Body*>::iterator bit = bodies.begin(); bit != bodies.end(); bit++)
        if ((*bit)->GetName() == body->GetName()) return true;
      return false;
}
void WAMGazebo::getPoses(std::vector<Pose3d>& poses)
{  
  poses.clear();
  poses.push_back(zeroBase);
  for(int i= JOINT1; i<= JOINT7; ++i)
  {
      poses.push_back(parent_->GetJoint(i)->GetJointBody(0)->GetRelativePose());
  }
}
void  WAMGazebo::getPosition(Joint* j1,double& pos)
{
	double d=0;
    d = j1->GetAngle(0).GetAsRadian();
	d*=1000;
	d=round(d);
	d/=1000;
	pos=d;
	if(pos < 0.01 && pos > -0.01)pos=0.0;
}
void WAMGazebo::publishFeed(int st)
{
	feedback feed;
	feed.header.stamp=ros::Time::now();
	feed.header.frame_id="wambase";
    feed.feedback.header.stamp=ros::Time::now();
	feed.feedback.header.frame_id="wambase";
	feed.feedback.joint_names=names_joints;
	getState(joints_positions);
	feed.feedback.actual.positions=joints_positions;
	feed.status.goal_id.stamp=ros::Time::now();
	feed.status.goal_id.id="wambase";
	
	switch(st)
	{
		case 0:  feed.status.status=actionlib_msgs::GoalStatus::PENDING;
		       break; 
		case 1:  feed.status.status=actionlib_msgs::GoalStatus::ACTIVE;
		       break; 
		case 2:  feed.status.status=actionlib_msgs::GoalStatus::PREEMPTED;
		       break; 
		case 3:  feed.status.status=actionlib_msgs::GoalStatus::SUCCEEDED;
		       break; 
		case 4:  feed.status.status=actionlib_msgs::GoalStatus::ABORTED;
		       break; 
		case 5:  feed.status.status=actionlib_msgs::GoalStatus::REJECTED;
		       break; 
		case 6:  feed.status.status=actionlib_msgs::GoalStatus::PREEMPTING;
		       break; 
		case 7:  feed.status.status=actionlib_msgs::GoalStatus::RECALLING;
		       break; 
		case 8:  feed.status.status=actionlib_msgs::GoalStatus::RECALLED;
		       break; 
		case 9:  feed.status.status=actionlib_msgs::GoalStatus::LOST;
		       break; 
	}
}


bool WAMGazebo::GenerealStateCB(iri_wam_common_msgs::GeneralState::Request& request,iri_wam_common_msgs::GeneralState::Response& response)
{
	if( 1 == request.index or 3 == request.index)
	{
	 //RobotState
	    getState(joints_positions);
	    std::vector<double> vel;
	    std::vector<double> force;
	    for(int ii=JOINT1; ii<JOINT7; ++ii)
	    {
	     vel.push_back(joints_[ii]->GetVelocity(0));
	    }
	    for(int ii=JOINT1; ii<JOINT7; ++ii)
	    {
	     force.push_back(joints_[ii]->GetForce(2));
	    }
	    response.robot_state.joint_state.header.frame_id="wambase";
	    response.robot_state.joint_state.header.stamp=ros::Time::now();
	    response.robot_state.joint_state.name=names_joints;
	    response.robot_state.joint_state.position=joints_positions;
	    response.robot_state.joint_state.velocity=vel;
	    response.robot_state.joint_state.effort=force; 
	}
	return true;
} 

