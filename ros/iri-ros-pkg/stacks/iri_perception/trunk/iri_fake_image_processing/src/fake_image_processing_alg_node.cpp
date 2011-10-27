#include "fake_image_processing_alg_node.h"

FakeImageProcessingAlgNode::FakeImageProcessingAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<FakeImageProcessingAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

FakeImageProcessingAlgNode::~FakeImageProcessingAlgNode(void)
{
  // [free dynamic memory]
}

void FakeImageProcessingAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void FakeImageProcessingAlgNode::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
		cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

	cv::imshow(WINDOW, cv_ptr->image);
	cv::waitKey(3);
	
	image_pub_.publish(cv_ptr->toImageMsg());
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void FakeImageProcessingAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void FakeImageProcessingAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<FakeImageProcessingAlgNode>(argc, argv, "fake_image_processing_alg_node");
}
