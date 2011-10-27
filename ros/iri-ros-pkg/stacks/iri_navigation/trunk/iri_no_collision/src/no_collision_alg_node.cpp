#include "no_collision_alg_node.h"
#include<tf/tf.h>

NoCollisionAlgNode::NoCollisionAlgNode(void) :
  move_base_aserver_(public_node_handle_, "MoveBase"),
  tf_listener_(ros::Duration(30.f)),
  is_odom_ready_(false),
  is_laser_ready_(false)
{
  //init class attributes if necessary
  loop_rate_ = 10;//in [Hz]

  // [init publishers]
  goal_marker_publisher_    = public_node_handle_.advertise<visualization_msgs::Marker>("goal_marker", 100);
  segway_cmd_publisher_     = public_node_handle_.advertise<geometry_msgs::Twist>("segway_cmd", 100);
  
  // [init subscribers]
  frontal_laser_subscriber_ = public_node_handle_.subscribe("scan", 100, &NoCollisionAlgNode::frontal_laser_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  move_base_aserver_.registerStartCallback(boost::bind(&NoCollisionAlgNode::startCallback, this, _1));
  move_base_aserver_.registerStopCallback(boost::bind(&NoCollisionAlgNode::stopCallback, this));
  move_base_aserver_.registerIsFinishedCallback(boost::bind(&NoCollisionAlgNode::isFinishedCallback, this));
  move_base_aserver_.registerHasSucceedCallback(boost::bind(&NoCollisionAlgNode::hasSucceedCallback, this));
  move_base_aserver_.registerGetResultCallback(boost::bind(&NoCollisionAlgNode::getResultCallback, this, _1));
  move_base_aserver_.registerGetFeedbackCallback(boost::bind(&NoCollisionAlgNode::getFeedbackCallback, this, _1));

  // [init action clients]

  goal_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  goal_marker_.type = visualization_msgs::Marker::CYLINDER;
  goal_marker_.action = visualization_msgs::Marker::ADD;
  goal_marker_.scale.x = 0.5;
  goal_marker_.scale.y = 0.5;
  goal_marker_.scale.z = 0.5;
  goal_marker_.color.a = 1.0;
  goal_marker_.color.r = 0.0;
  goal_marker_.color.g = 1.0;
  goal_marker_.color.b = 0.0;
  
}

NoCollisionAlgNode::~NoCollisionAlgNode(void)
{
  // [free dynamic memory]
}

void NoCollisionAlgNode::mainNodeThread(void)
{
  alg_.lock();
    if(!move_base_aserver_.isStarted() && is_laser_ready_)//is_odom_ready_
    {
      move_base_aserver_.start();
      ROS_INFO("NoCollisionAlgNode:: Server Started!"); 
    }
      
    // [fill msg structures]

    // [fill srv structure and make request to the server]
    
    // [fill action structure and make request to the action server]

    // [publish messages]

  alg_.unlock();
}

void NoCollisionAlgNode::frontal_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
//   ROS_INFO("NoCollisionAlgNode::frontal_laser_callback: New Message Received"); 

  alg_.lock();
    is_laser_ready_ = true;

    scan_.header          = msg->header;
    scan_.angle_min       = msg->angle_min;
    scan_.angle_max       = msg->angle_max;
    scan_.angle_increment = msg->angle_increment;
    scan_.time_increment  = msg->time_increment; 
    scan_.scan_time       = msg->scan_time;
    scan_.range_min       = msg->range_min;
    scan_.range_max       = msg->range_max;
    scan_.ranges          = msg->ranges;
    scan_.intensities     = msg->intensities;
   
  alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */
void NoCollisionAlgNode::startCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal)
{
  ROS_INFO("NoCollisionAlgNode::startCallback");


    // get goal pose request by client
    local_goal_pose_.header = goal->target_pose.header;
    local_goal_pose_.pose   = goal->target_pose.pose;

    // reset distance to goal for new action
    alg_.resetDistance2Goal();

    // get current time
    action_start_ = ros::Time::now();
}

void NoCollisionAlgNode::stopCallback(void)
{
  ROS_WARN("NoCollisionAlgNode::stopCallback!!");
  //lock access to driver if necessary
  alg_.lock();
      geometry_msgs::Twist twist;

      // send twist to platform
      segway_cmd_publisher_.publish(twist);
  //lock access to driver if necessary
  alg_.unlock();
}

bool NoCollisionAlgNode::isFinishedCallback(void)
{
  bool ret;

    ros::Duration elapsed_time = action_start_ - ros::Time::now();
    ret = ( alg_.isGoalReached() || elapsed_time.toSec() > ACTION_TIMEOUT );

  return ret;
}

bool NoCollisionAlgNode::hasSucceedCallback(void)
{
  bool ret;
    
    ret = alg_.isGoalReached();
  
  return ret;
}

void NoCollisionAlgNode::getResultCallback(move_base_msgs::MoveBaseResultPtr& result)
{
  //lock access to driver if necessary
  alg_.lock();
  
  //lock access to driver if necessary
  alg_.unlock();
}

void NoCollisionAlgNode::getFeedbackCallback(move_base_msgs::MoveBaseFeedbackPtr& feedback)
{
//   alg_.lock();
    ROS_INFO("NoCollisionAlgNode::getFeedbackCallback: NO_COL Action Alive!");
    try
    {
      std::string tf_prefix;
      public_node_handle_.param<std::string>("tf_prefix", tf_prefix, "");

      std::string source = local_goal_pose_.header.frame_id; //(tf_prefix+"/base_link");
      std::string target(tf_prefix+"/base_link");
      std::string fixed(tf_prefix+"/odom");

      ros::Time now = ros::Time(0);//ros::Time::now();
      //waitForTransform(target_frame, source_frame, time, timeout, polling_sleep_duration);
      bool tf_exists = tf_listener_.waitForTransform(target, source, now,
                                                     ros::Duration(1), 
                                                     ros::Duration(0.01)); 
      if(tf_exists)
      {
        geometry_msgs::PoseStamped current_local_goal;

        //transformPose(target_frame, target_time, pose_in, fixed_frame, pose_out);
        tf_listener_.transformPose(target, now, local_goal_pose_, fixed, current_local_goal);

        // send nav goal marker for rviz visualization
        goal_marker_.header.seq      = local_goal_pose_.header.seq;
        goal_marker_.header.stamp    = now;
        goal_marker_.header.frame_id = target;
        goal_marker_.pose            = current_local_goal.pose;
        goal_marker_publisher_.publish(goal_marker_);

        // update feedback with distance to goal
        feedback->base_position.header = current_local_goal.header;
        feedback->base_position.pose   = current_local_goal.pose;

        // compute new twist based on laser, odom and goal
        geometry_msgs::Twist twist = alg_.movePlatform(scan_, current_local_goal.pose);

        // send twist to platform
        segway_cmd_publisher_.publish(twist);
      }
      else
      {
        ROS_WARN("No transform: %s-->%s",source.c_str(), target.c_str());
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }

//   alg_.unlock();
}

/*  [action requests] */

void NoCollisionAlgNode::node_config_update(Config &config, uint32_t level)
{
}

void NoCollisionAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<NoCollisionAlgNode>(argc, argv, "no_collision_alg_node");
}

