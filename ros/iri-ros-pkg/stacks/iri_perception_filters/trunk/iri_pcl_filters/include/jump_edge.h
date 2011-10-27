#ifndef _FILTER_JUMP_EDGE_H_
#define _FILTER_JUMP_EDGE_H_

// [Eigen]
#include "Eigen/Core"

// [ROS]
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "image_transport/image_transport.h" 

/* // [OpenCV] */
/* #include <cv_bridge/cv_bridge.h> */
/* #include <opencv2/imgproc/imgproc.hpp> */
/* #include <opencv2/highgui/highgui.hpp> */

// [publisher subscriber headers]
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

// [services]

// static const char WINDOW[] = "Image window";

class FilterJE {
    public:
      FilterJE();
      ~FilterJE();
     
    private:
      sensor_msgs::Image ImageMessage_msg;
      
      ros::NodeHandle nh;

      // [Subscribers]
      ros::Subscriber pcl2_sub; // Point Cloud 2

      // [Publishers]
      ros::Publisher pcl2_pub;  // Point Cloud 2
      image_transport::ImageTransport it; 
      image_transport::Publisher jei_pub; // Depth Image
      image_transport::Publisher jeip_pub; // Depth Image

      // [Services]
      ros::ServiceServer jef_srv;
      
      // Intrinsec Parameters
      double sx, sy, u0, v0, skw, ang_thr;

      void pcl2_sub_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};
#endif
