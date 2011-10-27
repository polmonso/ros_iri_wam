/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: ROS interface to a Position2d controller for a Differential drive.
 * Author: Daniel Hewlett (adapted from Nathan Koenig)
 */

#include <algorithm>
#include <assert.h>

#include "teo_plugin.h"

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/Quatern.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Body.hh>
#include <gazebo/World.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/PhysicsEngine.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("teo_plugin", TeoPlugin);

enum
{
  FR, FL, BR, BL
};

// Constructor
TeoPlugin::TeoPlugin(Entity * parent) :
    Controller(parent),
    vel_x_(0.0),
    rot_z_(0.0),
    previous_displ_(0.0),
    alive_(true)
{
  parent_ = dynamic_cast<Model *> (parent);

  if (!parent_)
    gzthrow("Differential_Position2d controller requires a Model as its parent");

  prevUpdateTime_ = Simulator::Instance()->GetSimTime();

  Param::Begin(&parameters);
  robotNamespaceP = new ParamT<std::string> ("robotNamespace", "/", 0);
  topicNameP      = new ParamT<std::string> ("topicName", "", 1);
  frNameP         = new ParamT<std::string> ("fr_joint","", 1);
  flNameP         = new ParamT<std::string> ("fl_joint","", 1);
  brNameP         = new ParamT<std::string> ("br_joint","", 1);
  blNameP         = new ParamT<std::string> ("bl_joint","", 1);
  Param::End();
}

// Destructor
TeoPlugin::~TeoPlugin()
{
    delete robotNamespaceP;
    delete topicNameP;
    delete frNameP;
    delete flNameP;
    delete brNameP;
    delete blNameP;
    delete callback_queue_thread_;
    delete rosnode_;
    delete transform_broadcaster_;
}

// Load the controller
void TeoPlugin::LoadChild(XMLConfigNode *node)
{
  pos_iface_ = dynamic_cast<libgazebo::PositionIface*> (GetIface("position"));

  // Initialize the ROS node and subscribe to cmd_vel

  robotNamespaceP->Load(node);
  topicNameP->Load(node);
  frNameP->Load(node);
  flNameP->Load(node);
  brNameP->Load(node);
  blNameP->Load(node);

  robotNamespace = robotNamespaceP->GetValue();
  topicName      = topicNameP->GetValue();

  ROS_INFO("fr: '%s'",frNameP->GetValue().c_str());
  ROS_INFO("fl: '%s'",flNameP->GetValue().c_str());
  ROS_INFO("br: '%s'",brNameP->GetValue().c_str());
  ROS_INFO("bl: '%s'",blNameP->GetValue().c_str());

  joints_[FR] = parent_->GetJoint(** frNameP);
  joints_[FL] = parent_->GetJoint(** flNameP);
  joints_[BR] = parent_->GetJoint(** brNameP);
  joints_[BL] = parent_->GetJoint(** blNameP);

  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "diff_drive_plugin", 
            ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(robotNamespace);

  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(topicName, 1,
                                                          boost::bind(&TeoPlugin::cmdVelCallback, this, _1),
                                                          ros::VoidPtr(), &queue_);
  sub_ = rosnode_->subscribe(so);
  pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);
}

// Initialize the controller
void TeoPlugin::InitChild()
{
  // Reset odometric pose
  ROS_INFO("init child");
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;

  callback_queue_thread_ = new boost::thread(boost::bind(&TeoPlugin::QueueThread, this));
}

// Reset
void TeoPlugin::ResetChild()
{
  // Reset odometric pose
  ROS_INFO("reset child");
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;
}

// Update the controller
void TeoPlugin::UpdateChild()
{
  // TODO: Step should be in a parameter of this function
  double wd, ws;
  double d1, d2;
  double dr, da;
  Time delta_time;

  //myIface->Lock(1);

  // GetPositionCmd();

  // wd = **(wheelDiamP);
  // ws = **(wheelSepP);

  //stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime();
  delta_time = Simulator::Instance()->GetSimTime() - prevUpdateTime_;
  prevUpdateTime_ = Simulator::Instance()->GetSimTime();

  // Distance travelled by front wheels
  //d1 = stepTime.Double() * wd / 2 * joints[LEFT]->GetVelocity(0);
  //d2 = stepTime.Double() * wd / 2 * joints[RIGHT]->GetVelocity(0);

  //dr = (d1 + d2) / 2;
  // da = (d1 - d2) / ws;

  double displacement_x = delta_time.Double() * vel_x_;

  /* if (vel_x_ > 0.0) {
      ROS_INFO("vel_x_: '%f'", vel_x_);
      ROS_INFO("time  : '%f'", delta_time.Double());
      ROS_INFO("displ : '%f'", displacement_x);
  }
  */
  /* Compute odometric pose
  odomPose[0] += dr * cos(odomPose[2]);
  odomPose[1] += dr * sin(odomPose[2]);
  odomPose[2] += da;

  odomPose[0] += dr * cos(odomPose[2]);
  odomPose[1] += dr * sin(odomPose[2]);
  odomPose[2] += da; */


    odomPose[0] += displacement_x;
    odomPose[1] += 0;
    odomPose[2] += 0;

    odomVel[0] = vel_x_;
    odomVel[1] = 0.0;
    odomVel[2] = 0.0;

    // ROS_DEBUG("updating odomPose[0]: '%f'", odomPose[0]);

    ROS_DEBUG("odomVel[0]: '%f'", odomVel[0]);
    ROS_DEBUG("vel X: '%f'", vel_x_);

    joints_[FR]->SetVelocity(0, vel_x_);
    joints_[FL]->SetVelocity(0, vel_x_);
    joints_[BR]->SetVelocity(0, vel_x_);
    joints_[BL]->SetVelocity(0, vel_x_);

    joints_[FR]->SetMaxForce(0, 5);
    joints_[FL]->SetMaxForce(0, 5);
    joints_[BR]->SetMaxForce(0, 5);
    joints_[BL]->SetMaxForce(0, 5);

    write_position_data();
    publish_odometry();

    previous_displ_ += odomPose[0];

  //myIface->Unlock();
}

// Finalize the controller
void TeoPlugin::FiniChild()
{
  //std::cout << "ENTERING FINALIZE\n";
  alive_ = false;
  // Custom Callback Queue
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_->join();
  //std::cout << "EXITING FINALIZE\n";
}

void TeoPlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  lock.lock();

  vel_x_  = cmd_msg->linear.x;
  rot_z_  = cmd_msg->angular.z;

  ROS_INFO("cmd_vel callback recieved - X: '%f'", vel_x_);

  lock.unlock();
}

// NEW: custom callback queue thread
void TeoPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (alive_ && rosnode_->ok())
  {
    //    std::cout << "CALLING STUFF\n";
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// NEW: Update this to publish odometry topic
void TeoPlugin::publish_odometry()
{
  // get current time
  ros::Time current_time_((Simulator::Instance()->GetSimTime()).sec, (Simulator::Instance()->GetSimTime()).nsec); 

  // getting data for base_footprint to odom transform
  btQuaternion qt;
  // TODO: Is there something wrong here? RVIZ has a problem?
  qt.setEulerZYX(pos_iface_->data->pose.yaw, pos_iface_->data->pose.pitch, pos_iface_->data->pose.roll);
  btVector3 vt(pos_iface_->data->pose.pos.x, pos_iface_->data->pose.pos.y, pos_iface_->data->pose.pos.z);
  tf::Transform base_footprint_to_odom(qt, vt);

  transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                            current_time_,
                                                            "odom",
                                                            "base_footprint"));

  // publish odom topic
  odom_.pose.pose.position.x = pos_iface_->data->pose.pos.x;
  odom_.pose.pose.position.y = pos_iface_->data->pose.pos.y;

  gazebo::Quatern rot;
  rot.SetFromEuler(gazebo::Vector3(pos_iface_->data->pose.roll, pos_iface_->data->pose.pitch, pos_iface_->data->pose.yaw));

  odom_.pose.pose.orientation.x = rot.x;
  odom_.pose.pose.orientation.y = rot.y;
  odom_.pose.pose.orientation.z = rot.z;
  odom_.pose.pose.orientation.w = rot.u;

  odom_.twist.twist.linear.x = pos_iface_->data->velocity.pos.x;
  odom_.twist.twist.linear.y = pos_iface_->data->velocity.pos.y;
  odom_.twist.twist.angular.z = pos_iface_->data->velocity.yaw;

  odom_.header.frame_id = tf::resolve(tf_prefix_, "odom");
  odom_.child_frame_id = "base_footprint";

  //odom_.header.stamp = current_time_;
  odom_.header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
  odom_.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;

  ROS_DEBUG("Odometry Position - X: '%f' Y: '%f'",  odom_.pose.pose.position.x,  odom_.pose.pose.position.y);
  pub_.publish(odom_);
}

// Update the data in the interface
void TeoPlugin::write_position_data()
{
  // TODO: Data timestamp
  pos_iface_->data->head.time = Simulator::Instance()->GetSimTime().Double();
  ROS_DEBUG("write_position_data - X: '%f', - Y '%f', - Z '%f'", odomPose[0], odomPose[1], NORMALIZE(odomPose[2]));
  ROS_DEBUG("write_vel_data - X: '%f', - Y '%f'", odomVel[0], odomVel[2]);

  pos_iface_->data->pose.pos.x = odomPose[0];
  pos_iface_->data->pose.pos.y = odomPose[1];
  pos_iface_->data->pose.yaw = NORMALIZE(odomPose[2]);

  pos_iface_->data->velocity.pos.x = odomVel[0];
  pos_iface_->data->velocity.yaw = odomVel[2];

  // TODO
  pos_iface_->data->stall = 0;
}
