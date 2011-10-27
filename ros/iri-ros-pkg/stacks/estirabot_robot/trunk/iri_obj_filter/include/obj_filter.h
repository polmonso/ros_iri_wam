#ifndef _OBJECT_FILTER_H_
#define _OBJECT_FILTER_H_

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "iri_wam_common_msgs/compute_obj_grasp_pose.h"

#include "tf/tf.h"
#include <tf/transform_broadcaster.h>

// [PCL_ROS]
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/register_point_struct.h"

//pcl::toROSMsg
#include <pcl/io/pcd_io.h>


// [publisher subscriber headers]
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "normal_descriptor_node/heat_map.h"
#include "normal_descriptor_node/wrinkle.h"

#include "termios.h"
#include "thread.h"
#include "socket.h"
#include "socketclient.h"
#include "mutex.h"
#include "thread.h"
#include "eventexceptions.h"

#define AZONE 1000
#define BZONE 2000
#define ANYZONE 0

typedef union {
    struct /*anonymous*/
    {
      unsigned char Blue;
      unsigned char Green;
      unsigned char Red;
      unsigned char Alpha;
    };
    float float_value;
    long long_value;
} RGBValue;

class ObjectFilter {
    public:
      ObjectFilter();
      void mainLoop(void);
    private:

      int ncols;
      int nrows;
      int focused_obj;
      geometry_msgs::PoseStamped graspPose;
      sensor_msgs::PointCloud2ConstPtr* msg_;
      pcl::PointCloud<pcl::PointXYZRGB> pcl_xyzrgb;
      float* xi; // image x coordinates of pcl2
      float* yi; // image y coordinates of pcl2
      float* zi; // image z coordinates of pcl2
      float* ci; // image color coordinates of pcl2

      tf::TransformBroadcaster tf_br;
      tf::Transform transform_grasping_point;

      // ROS Handlers
      ros::NodeHandle nh;

      // ROS Publishers
      ros::Publisher filtered_pcl2_publisher;

      // ROS Subscribers
      ros::Subscriber pcl2_sub;
      ros::Subscriber segmented_img_sub;
      ros::Subscriber heat_map_sub;
      ros::Subscriber focused_object_sub;

      // ROS Services
      ros::ServiceClient wrinkle_client;
      ros::ServiceServer compute_grasp;
      normal_descriptor_node::wrinkle wrinkle_srv;


      void pcl2_sub_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
      void heat_map_sub_callback(const normal_descriptor_node::heat_map& msg);
      void focused_obj_sub_callback(const std_msgs::Int32& msg);
      void segmented_img_sub_callback(const sensor_msgs::ImageConstPtr& msg);

      bool compute_grasp_pose_callback(iri_wam_common_msgs::compute_obj_grasp_pose::Request &req, iri_wam_common_msgs::compute_obj_grasp_pose::Response &res);
};
#endif
