#include "average_point_cloud_alg_node.h"
#include <boost/foreach.hpp>


AveragePointCloudAlgNode::AveragePointCloudAlgNode(void) :iBuffer (5), iCurrent (0), init (false)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  
  //TODO:read this value!!
  cloudBuffer.resize(iBuffer);
  // [init publishers]
  //  this->output_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("output", 5);
  this->output_publisher_ = this->public_node_handle_.advertise<PointCloud>("output", 5);
  
  // [init subscribers]
  //  this->input_subscriber_ = this->public_node_handle_.subscribe("input", 5, &AveragePointCloudAlgNode::input_callback, this);
  this->input_subscriber_ = this->public_node_handle_.subscribe<PointCloud>("input", 5, &AveragePointCloudAlgNode::input_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

AveragePointCloudAlgNode::~AveragePointCloudAlgNode(void)
{
  // [free dynamic memory]
}

void AveragePointCloudAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //this->PointCloud2_msg.data = my_var;
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  //this->output_publisher_.publish(this->PointCloud2_msg_);
  //this->output_publisher_.publish(this->PointCloud_msg_);
}

/*  [subscriber callbacks] */
//void AveragePointCloudAlgNode::input_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) 
void AveragePointCloudAlgNode::input_callback(const PointCloud::ConstPtr& msg) 
{ 

  PointCloud cloud3_;
  if (output_publisher_.getNumSubscribers () > 0) 
  {

    //ROS_INFO("AveragePointCloudAlgNode::input_callback: New Message Received %d %d %d", iCurrent, msg->width, msg->height); 
    
    //    PointCloud PointCloud_msg_ (*msg);
    //    PointCloud PointCloud_msg_ =*msg;
    //copyPointCloud(*msg,PointCloud_msg_); 
    PointCloud_msg_.width  = msg->width ;
    PointCloud_msg_.height = msg->height;
    PointCloud_msg_.points.resize (msg->width * msg->height);
    PointCloud_msg_.header   = msg->header;
    PointCloud_msg_.is_dense = msg->is_dense;


//////////  
    if (!init) {
//      ROS_INFO("INIT"); 
      //      cloud2_[iCurrent] = *msg;
      //copyPointCloud(*msg,cloud2_[iCurrent]);  
      cloud3_.width  = msg->width;
      cloud3_.height = msg->height;
      cloud3_.points.resize (msg->width * msg->height);   
      cloud3_.header   = msg->header;
      for (size_t i = 0; i < msg->points.size (); ++i) {
	cloud3_.points[i].x = msg->points[i].x;
	cloud3_.points[i].y = msg->points[i].y;
	cloud3_.points[i].z = msg->points[i].z;
      }
      this->alg_.lock();
      cloudBuffer.push_back(cloud3_);
      if (++iCurrent == iBuffer) {
	init = true;
	iCurrent = 0;
      }
      this->alg_.unlock();
    } else {
//      ROS_INFO("AVERAGING"); 
      //      cloud2_[iCurrent] = *msg;
      //copyPointCloud(*msg,cloud2_[iCurrent]);
      cloud3_.width  = msg->width;
      cloud3_.height = msg->height;
      cloud3_.points.resize (msg->width * msg->height);
      cloud3_.header   = msg->header;
      for (size_t i = 0; i < msg->points.size (); ++i) {
	cloud3_.points[i].x = msg->points[i].x;
	cloud3_.points[i].y = msg->points[i].y;
	cloud3_.points[i].z = msg->points[i].z;
      }
      this->alg_.lock();

      cloudBuffer.push_back(cloud3_);
      //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
      for (size_t j=0;j<msg->height;j++) {
	for (size_t i=0;i<msg->width;i++) {
          PointCloud_msg_(i,j).x = 0;
          PointCloud_msg_(i,j).y = 0;
          PointCloud_msg_(i,j).z = 0;
	  for (int k=0;k<iBuffer;k++) {
	    this->PointCloud_msg_(i,j).x += cloudBuffer[k].at(i,j).x/(iBuffer);
	    this->PointCloud_msg_(i,j).y += cloudBuffer[k].at(i,j).y/(iBuffer);
	    this->PointCloud_msg_(i,j).z += cloudBuffer[k].at(i,j).z/(iBuffer);
	  }
	}
      }
      this->alg_.unlock();
      this->output_publisher_.publish(this->PointCloud_msg_);
      iCurrent = (iCurrent +1 ) % iBuffer;
    }
  }
 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void AveragePointCloudAlgNode::node_config_update(Config &config, uint32_t level)
{
  //update driver with new_cfg data
    this->alg_.lock();
    ROS_INFO ("Reconfigure request :%d, level %d", config.iBuffer, level);
    try
    {
      this->iBuffer = config.iBuffer;
      cloudBuffer.resize(iBuffer);
      init =false;
    }
    catch (CException & e)
    {
      std::cout << e.what () << std::endl;
    }
    this->alg_.unlock();
  // save the current configuration
  //this->config_ = config;
}

void AveragePointCloudAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<AveragePointCloudAlgNode>(argc, argv, "average_point_cloud_alg_node");
}
