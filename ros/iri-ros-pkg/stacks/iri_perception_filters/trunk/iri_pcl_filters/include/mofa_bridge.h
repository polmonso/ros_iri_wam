#ifndef _MOFA_BRIDGE_H_
#define _MOFA_BRIDGE_H_

#include "termios.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <set>

#include "cv.h"
#include "highgui.h"

#include "ros/ros.h"
#include "ros/callback_queue.h"

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
#include "geometry_msgs/PoseStamped.h"
#include "iri_wam_common_msgs/obs_request.h"

//vanila socket
#include "socket.h"
#include "socketclient.h"
#include "mutex.h"
#include "thread.h"
#include "eventexceptions.h"
#include <arpa/inet.h>

#define ERROR_POS 0
#define RX_ID_POS 1
#define MAXCONN 5
#define BUFFERSIZE 512
#define DATASIZE 512

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

class MofaBridge {
    public:
      MofaBridge();
//TODO close socket
      void open();
    private:

      //vanila socket
      bool connected;
      std::string serverip;
      int port;
      ///mutex protecting writes on the socket
      CMutex socketmutex;

      ///Socket server that accepts the WAM connection only
      CSocketClient *csocket;

      ///Eventserver (singleton) 
      CEventServer *eventserver;

      /// list of internal events
      std::list<std::string> events;

      int num_objects;
      std::set<int> numobjAset;
      std::set<int> numobjBset;
      std::set<int> numobjset;
 
      //service activation parameters
      bool flag_mpcl2;

      int idx;
      int nr;
      int nc;
      sensor_msgs::PointCloud2ConstPtr* msg_;
      pcl::PointCloud<pcl::PointXYZRGB> pcl_xyzrgb;
      float* xi; // image x coordinates of pcl2
      float* yi; // image y coordinates of pcl2
      float* zi; // image z coordinates of pcl2
      float* ci; // image color coordinates of pcl2

      float* ai; // amplitude image array pointer

      std::ofstream matlab_xyzrgb;
      std::ifstream matlab_xyzrgb_in;

      // ROS Handlers
      ros::NodeHandle nh;

      // ROS Publishers
      ros::Publisher labeled_pcl2_publisher;

      // ROS Subscribers
      ros::Subscriber pcl2_sub;

      // ROS Services
      ros::ServiceServer obs_request;
      
      void pcl2_sub_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
      bool obs_request_callback(iri_wam_common_msgs::obs_request::Request &req, iri_wam_common_msgs::obs_request::Response &res);
      
      void reloadRGB();
};
#endif
