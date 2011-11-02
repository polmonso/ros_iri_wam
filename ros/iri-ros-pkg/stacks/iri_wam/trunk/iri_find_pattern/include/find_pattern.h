#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "geometry_msgs/PoseStamped.h" 
#include "ros/this_node.h"
#include "ros/names.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include <Eigen/StdVector>
#include <Eigen/Dense>

#include <tf/transform_broadcaster.h>
#include "tf/tfMessage.h"

#define BOARD_W 5
#define BOARD_H 8
#define BOARDTOTAL 40
#define SQUARESIZE 29.94444

//TODO retrieve from parameter server
//kinect highres
static float Hintrinsic_data[9] =
{1046.82,    -0.560873,      606.300,
 0.00000,      1047.16,      549.347,
 0.00000,      0.00000,      1.00000};
//{1052.28,    -0.709805,      605.432,
//0.00000,      1053.43,      547.677,
//0.00000,      0.00000,      1.00000};

//kinect2
//static float Hintrinsic_data[9] =
//{524.139597, 0.000000, 523.633543,
// 0.000000, 306.258531, 277.906362,
// 0.000000, 0.000000, 1.000000};
//kinect
//static float Hintrinsic_data[9] =
//{1056.884243, 0.000000, 606.291412, 0.000000, 1060.512216, 542.698832, 0.000000, 0.000000, 1.000000};
//static float Hintrinsic_data[9] =
//      {1646.52,      4.96473,      629.935,
//      0.00000,      1645.35,      473.293,
//      0.00000,      0.00000,      1.00000};

//kinect highres
static float Kdistortion_data[4] =
{0.157871,-0.285724,0,0};
//{0.098301, -0.175114, 0.002126, 0.006032};// 0.0000
//kinect2
//static float Kdistortion_data[4] =
//{0.140907, -0.226361, 0.002334, 0.002312}; // 0.0000
//kinect
//static float Kdistortion_data[4] =
//{0.179540, -0.300929, 0.001850, -0.000358}; //, 0.0000}
//static float Kdistortion_data[4] =
//            {-0.211152,     0.202376,      0.00000, 0.00000}; 

//handeye part
//Hcam2world
//static float Hgrid2marker_data[16] =
//   {0.7367,    0.1376,   -0.6621,   -0.0327,
//    0.1617,   -0.9865,   -0.0251,    0.0576,
//   -0.6566,   -0.0886,   -0.7490,    0.1999,
//         0,         0,         0,    1.0000};

//static float Hmarker2grid_data[16] =
static float Hgrid2marker_data[16] =
    {0.0321550,  -0.00962044,     0.999436,     -55.5281,
    -0.999473,  -0.00475892,    0.0321104,      15.1205,
   0.00444731,    -0.999942,  -0.00976842,      6.65012,
      0.00000,      0.00000,      0.00000,      1.00000};

//Hgrid2marker
//to opennicamera with opt^H_c = [0 -1 0 -0.04; 0 0 -1 0; 1 0 0 0; 0 0 0 1]
static float Hcam2world_data[16] =
{  -0.1969,   -0.0145,   -0.9803,    0.9738,
    0.0414,   -0.9991,    0.0064,   -0.0705,
   -0.9795,   -0.0393,    0.1974,    0.4614,
         0,         0,         0,    1.0000};
//   {-0.3028,   -0.0718,   -0.9504,  0.8930161,
//    0.0835,   -0.9953,    0.0486,  -0.0403882,
//   -0.9494,   -0.0646,    0.3073,  0.3208564,
//         0,         0,         0,    1.0000};

//to rgb_optical
//  {-0.0425713,     0.912537,    -0.406772,      1.00945,
//     0.998769,    0.0492444,   0.00594514,      0.0272568,
//    0.0254565,    -0.406018,    -0.913510,      0.315302,
//      0.00000,      0.00000,      0.00000,      0.001};

class FindPattern {

  public:
    FindPattern(ros::NodeHandle &n);
    ~FindPattern();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr);
    void mainLoop();

  protected:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    sensor_msgs::CvBridge bridge_;
    image_transport::Publisher image_pub_;

    ros::Publisher tf_publisher_pattern_pose_;
    ros::Publisher tf_publisher_pattern_;
    ros::Publisher tf_publisher_;
    tf::tfMessage tfMessage_msg_;
    geometry_msgs::PoseStamped poseStamped_msg_;

  private:
     bool retrieveMatrixFromParameterServer(std::string paramname, std::vector<double> aux_vector);
     bool findPatternPose(IplImage *cv_image);

     tf::TransformBroadcaster tf_br;
     /// Handeye extrinsic camera parameters matrix
     CvMat cvHcam2world;
     CvMat cvHgrid2marker;
     tf::Transform Hcam2world;
     tf::Transform Hgrid2marker;

     /// Intrinsic camera parameters matrix
     CvMat Hintrinsic;

     /// distortion coeficients array
     CvMat Kdistortion;

     /// undistortion map x
     IplImage* mapx;
     /// undistortion map y
     IplImage* mapy;

     IplImage* gray;

     /** pattern chasing stuff*/
     CvMat* image_points;
     CvMat* object_points;
     CvMat* intrinsic_matrix;
     CvMat* distortion_coeffs;

};
