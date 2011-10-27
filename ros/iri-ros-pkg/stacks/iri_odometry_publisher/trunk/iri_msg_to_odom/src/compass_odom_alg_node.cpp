#include "compass_odom_alg_node.h"

CompassOdomAlgNode::CompassOdomAlgNode(void)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  this->public_node_handle_.param<std::string>("parent_id", parent_id_, "odom");
  alg_.parent_id_ = parent_id_;
  this->public_node_handle_.param<bool>("publish_tf", publish_tf_, false);
  this->public_node_handle_.param<bool>("publish_imu", publish_imu_, false);

  // [init publishers]
  this->compass_odom_publisher_ = this->public_node_handle_.advertise<nav_msgs::Odometry>("compass_odom", 100);
  if(publish_imu_)
    this->compass_imu_publisher_ = this->public_node_handle_.advertise<sensor_msgs::Imu>("compass_imu", 100);

  // [init subscribers]
  this->compass_subscriber_ = this->public_node_handle_.subscribe("compass", 100, &CompassOdomAlgNode::compass_callback, this);

  // Events
  ES = CEventServer::instance();
  std::stringstream const_time;
  const_time << ros::Time::now();
  TCM3_IN = "compass3axis_in_" + const_time.str();
  ES->create_event(TCM3_IN);
  SUBS_LIST.push_back(TCM3_IN);
  compass_arrived_ = false;

  // UI

  ROS_WARN("Mounting Position MUST BE 6, otherwise changes in transformation are required!");
  ROS_INFO("Publish TF: %d",publish_tf_);
  ROS_INFO("Publish IMU: %d",publish_imu_);
  ROS_INFO("Parent ID: %s", parent_id_.c_str());
  //ROS_INFO("Frame ID: %s",  frame_id_);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

CompassOdomAlgNode::~CompassOdomAlgNode(void)
{
  // [free dynamic memory]
}

void CompassOdomAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //this->Odometry_msg.data = my_var;

  ES->wait_all(SUBS_LIST);
  alg_.getOdometry(Odometry_msg_,Transform_msg_);

  if(publish_imu_)
    alg_.getImuMessage(Odometry_msg_,Imu_msg_);

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->compass_odom_publisher_.publish(Odometry_msg_);

  if(publish_imu_)
    this->compass_imu_publisher_.publish(Imu_msg_);

  if(publish_tf_)
    compass_odom_broadcaster_.sendTransform(Transform_msg_);
}

/*  [subscriber callbacks] */
void CompassOdomAlgNode::compass_callback(const iri_common_drivers_msgs::compass3axis & msg)
{
  //ROS_INFO("CompassOdomAlgNode::compass_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary

  this->compass_mutex_.enter();

  //ROS_INFO("compass msg: %f %f %f",msg.angles[0],msg.angles[1],msg.angles[2]);

  alg_.lock();
    if(compass_arrived_){
      alg_.old_compass_ = alg_.new_compass_;
      alg_.new_compass_ = msg;
      ES->set_event(TCM3_IN);
    }else{
      alg_.new_compass_ = msg;
      compass_arrived_ = true;
    }
    // NOMES PER EXPERIMENT USANT CONFIGURACIO 5 !
    //     alg_.new_compass_.angles[0] -= 90;
    //     double pitch = -alg_.new_compass_.angles[1];
    //     alg_.new_compass_.angles[1] = alg_.new_compass_.angles[2];
    //     alg_.new_compass_.angles[2] = pitch;
  alg_.unlock();

  //unlock previously blocked shared variables

  this->compass_mutex_.exit();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void CompassOdomAlgNode::node_config_update(Config &config, uint32_t level)
{
}

void CompassOdomAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<CompassOdomAlgNode>(argc, argv, "compass_odom_alg_node");
}
