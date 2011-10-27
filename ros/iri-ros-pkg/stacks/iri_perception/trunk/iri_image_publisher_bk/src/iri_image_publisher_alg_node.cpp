#include "iri_simple_perception_alg_node.h"

IriSimplePerceptionAlgNode::IriSimplePerceptionAlgNode(void) : imgtransport_(public_node_handle_)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  image_pub_ = imgtransport_.advertise("image", 1);

  this->filename_ = "default.jpg";
  cv_image.header.stamp = ros::Time::now();
  cv_image.header.frame_id = "camera";
  cv_image.encoding = "bgr8";

  ROS_INFO("Image reading");
  cv_image.image = cv::imread(this->filename_);
  ROS_INFO("Image read");

  if(cv_image.image.data==NULL){
    ROS_ERROR("Image %s could not be read", this->filename_.c_str());
    image_loaded = false;
  }else{
    ROS_INFO("Image %s read", this->filename_.c_str());
    image_loaded = true;
  } 

  ROS_INFO("Image read");
  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

IriSimplePerceptionAlgNode::~IriSimplePerceptionAlgNode(void)
{
  // [free dynamic memory]
}

void IriSimplePerceptionAlgNode::mainNodeThread(void)
{

  this->alg_.lock();
  if(image_loaded){

    // [fill msg structures]
    image_pub_.publish(cv_image.toImageMsg());
    
  }else{
  //  ROS_WARN("Image not loaded");
  }
  this->alg_.unlock();
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void IriSimplePerceptionAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  ROS_INFO("Image reading config");
  this->filename_ = config.filename;

  cv_image.header.stamp = ros::Time::now();
  cv_image.header.frame_id = "camera";
  cv_image.encoding = "bgr8";
  ROS_INFO("Image reading");
  cv_image.image = cv::imread(filename_, -1);

  if(cv_image.image.data==NULL){
    ROS_ERROR("Image %s could not be read", this->filename_.c_str());
    image_loaded = false;
  }else{
    ROS_INFO("Image %s read", this->filename_.c_str());
    image_loaded = true;
  }

  this->alg_.unlock();
}

void IriSimplePerceptionAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<IriSimplePerceptionAlgNode>(argc, argv, "iri_simple_perception_alg_node");
}
