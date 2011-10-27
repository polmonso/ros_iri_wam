#ifndef _BOUNDING_BOX_H_
#define _BOUNDING_BOX_H_

// [Eigen]
#include "Eigen/Core"

// [ROS]
#include "ros/ros.h"
#include "ros/callback_queue.h"

// [publisher subscriber headers]
#include "sensor_msgs/PointCloud2.h"

// [services]

// static const char WINDOW[] = "Image window";

class FilterBB {
    public:
      FilterBB();
      ~FilterBB();
     
    private:
      ros::NodeHandle nh;

      // [Subscribers]
      ros::Subscriber pcl2_sub; // Point Cloud 2

      // [Publishers]
      ros::Publisher pcl2_pub;  // Point Cloud 2

      // [Services]
      
      // Bounding Box Threshold
      double x_s, y_s, z_s, x_e, y_e, z_e;
     
      void pcl2_sub_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};
#endif
