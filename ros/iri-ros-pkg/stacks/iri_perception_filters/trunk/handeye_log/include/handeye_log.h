#ifndef _HANDEYE_LOG_H_
#define _HANDEYE_LOG_H_

#include "termios.h"

#include <fstream>
#include <sstream>
#include <iomanip>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_srvs/Empty.h"
#include "iri_wam_common_msgs/wamaction.h"

#include "tf/tf.h"

// [PCL_ROS]
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/register_point_struct.h"

//pcl::toROSMsg
#include <pcl/io/pcd_io.h>

// [publisher subscriber headers]
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PoseStamped.h"

// [opencv2]
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>

#include "iridrivers/eventserver.h"
#include "iridrivers/threadserver.h"

#include <boost/circular_buffer.hpp>
#include <Eigen/StdVector>

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

class HandeyeLog {

 private:
  //service activation parameters
  bool flag_tf_robot;
  bool flag_tf_pattern;
  bool flag_iimg;
  bool flag_ipng;
  bool flag_dimg;
  bool flag_mpcl2;
  bool flag_tf_robot_inverse;
  int num_captures_;

  int idx;
  int nr;
  int nc;

  sensor_msgs::PointCloud2ConstPtr* msg_;
  int pcl_type_;
  pcl::PointCloud<pcl::PointXYZRGB> pcl_xyzrgb_;

  pcl::PointCloud<pcl::PointXYZI> pcl_xyzi_;
  /* boost::circular_buffer<pcl::PointCloud<pcl::PointXYZI> > xyzi_cb_(pcl::PointCloud::VectorType); */

  typedef pcl::PointXYZI Point;
  typedef pcl::PointCloud<Point> PointCloud;
  boost::circular_buffer<PointCloud, Eigen::aligned_allocator<PointCloud> > cloudBuffer_;

  pcl::PointCloud<pcl::PointXYZ> pcl_xyz_;
  cv_bridge::CvImagePtr cv_ptr_;
  boost::circular_buffer<cv_bridge::CvImagePtr> iimageBuffer_;

  sensor_msgs::Image::ConstPtr intens_image_;

  std::ofstream matlab_file_;
  std::ofstream matlab_file_seq;
  std::ofstream img_depth_;
  std::ofstream img_intensity_;
  std::ofstream tf_robot_file_;
  std::ofstream tf_pattern_file_;

  btTransform tf_robot_;
  btTransform tf_pattern_;

  // ROS Handlers
  ros::NodeHandle nh;

  // ROS Subscribers
  ros::Subscriber pcl2_sub;
  ros::Subscriber iimg_sub;
  ros::Subscriber tf_robot_sub;
  ros::Subscriber tf_pattern_sub;

  // ROS Services
  ros::ServiceServer srv;

  // Event Identifiers
  std::string mpcl2_event_on_id_;
  std::string mpcl2_event_off_id_;
  std::string mpcl2_event_stop_id_;
  std::string iimg_event_on_id_;
  std::string iimg_event_off_id_;
  std::string iimg_event_stop_id_;
  std::string ipng_event_on_id_;
  std::string ipng_event_off_id_;
  std::string ipng_event_stop_id_;
  std::string dimg_event_on_id_;
  std::string dimg_event_off_id_;
  std::string dimg_event_stop_id_;
  std::string tf_robot_event_on_id_;
  std::string tf_robot_event_off_id_;
  std::string tf_robot_event_stop_id_;
  std::string tf_pattern_event_on_id_;
  std::string tf_pattern_event_off_id_;
  std::string tf_pattern_event_stop_id_;
  std::string tf_robot_inverse_event_on_id_;
  std::string tf_robot_inverse_event_off_id_;
  std::string tf_robot_inverse_event_stop_id_;

  // Threads Identifiers
  std::string mpcl2_thread_id_;
  std::string iimg_thread_id_;
  std::string ipng_thread_id_;
  std::string dimg_thread_id_;
  std::string tf_robot_thread_id_;
  std::string tf_pattern_thread_id_;
  std::string tf_robot_inverse_thread_id_;

  // Thread Server handler
  CThreadServer *thread_server;
  // Event Server handler
  CEventServer *event_server;

  void pcl2_sub_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

  void iimg_sub_callback(const sensor_msgs::ImageConstPtr& msg);

  void tf_robot_sub_callback(const geometry_msgs::PoseStampedConstPtr& msg);

  void tf_pattern_sub_callback(const geometry_msgs::PoseStampedConstPtr& msg);

  bool srv_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

 protected:

  static void *store_mpcl2_thread(void *param);
  static void *store_iimg_thread(void *param);
  static void *store_ipng_thread(void *param);
  static void *store_dimg_thread(void *param);
  static void *store_tf_robot_thread(void *param);
  static void *store_tf_pattern_thread(void *param);
  static void *store_tf_robot_inverse_thread(void *param);

 public:
  HandeyeLog();
  ~HandeyeLog();

};
#endif
