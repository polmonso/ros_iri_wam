#include "pose2d_odom_alg_node.h"

Pose2dOdomAlgNode::Pose2dOdomAlgNode(void)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->pose2dodom_publisher_ = this->public_node_handle_.advertise<nav_msgs::Odometry>("pose2d_odom", 100);

  // [init subscribers]
  this->pose2din_subscriber_ = this->public_node_handle_.subscribe("pose2d", 100, &Pose2dOdomAlgNode::pose2din_callback, this);

  // Events
  ES = CEventServer::instance();
  std::stringstream const_time;
  const_time << ros::Time::now();
  P2D_IN = "pose2d_in_" + const_time.str();
  ES->create_event(P2D_IN);
  SUBS_LIST.push_back(P2D_IN);
  pose2d_arrived_ = false;

  this->public_node_handle_.param<std::string>("parent_id", parent_id_, "odom");
  alg_.parent_id_ = parent_id_;
  this->public_node_handle_.param<std::string>("frame_id", frame_id_, "pose2d");
  alg_.frame_id_ = frame_id_;
  this->public_node_handle_.param<bool>("publish_tf", publish_tf_, true);

  ROS_INFO("Publish TF: %d",publish_tf_);
  ROS_INFO("Parent ID: %s", parent_id_.c_str());
  ROS_INFO("Frame ID: %s",  frame_id_.c_str());


  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

Pose2dOdomAlgNode::~Pose2dOdomAlgNode(void)
{
  // [free dynamic memory]
}

void Pose2dOdomAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //this->Odometry_msg.data = my_var;

  ES->wait_all(SUBS_LIST);
  alg_.getOdometry(Odometry_msg_,Transform_msg_);

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->pose2dodom_publisher_.publish(this->Odometry_msg_);

  if(publish_tf_)
    pose2dodom_broadcaster_.sendTransform(Transform_msg_);

}

/*  [subscriber callbacks] */
void Pose2dOdomAlgNode::pose2din_callback(const geometry_msgs::Pose2D& msg)
{
  //ROS_INFO("Pose2dOdomAlgNode::pose2din_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->driver_.lock();
  //this->pose2din_mutex_.enter();

  alg_.lock();
    if(pose2d_arrived_){
      alg_.old_pose2D_ = alg_.new_pose2D_;
      alg_.new_pose2D_ = msg;
      alg_.old_time_   = alg_.new_time_;
      alg_.new_time_   = ros::Time::now();
      ES->set_event(P2D_IN);
    }else{
      alg_.new_pose2D_ = msg;
      alg_.new_time_   = ros::Time::now();
      pose2d_arrived_ = true;
    }
  alg_.unlock();

  //std::cout << msg->data << std::endl;

  //unlock previously blocked shared variables
  //this->driver_.unlock();
  //this->pose2din_mutex_.exit();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void Pose2dOdomAlgNode::node_config_update(Config &config, uint32_t level)
{
}

void Pose2dOdomAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<Pose2dOdomAlgNode>(argc, argv, "pose2d_odom_alg_node");
}
