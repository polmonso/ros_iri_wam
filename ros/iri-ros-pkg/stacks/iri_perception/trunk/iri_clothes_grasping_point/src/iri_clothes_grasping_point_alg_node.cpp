#include "iri_clothes_grasping_point_alg_node.h"

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";
static const char WINDOW2[] = "Auxiliar window";

IriClothesGraspingPointAlgNode::IriClothesGraspingPointAlgNode(void) : imgtransport_(public_node_handle_) 
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  this->image_pub_ = imgtransport_.advertise("image/out", 1);
  this->image_sub_ = imgtransport_.subscribe("image/in", 1, &IriClothesGraspingPointAlgNode::imageCallback, this);

  cv::namedWindow(WINDOW);
  cv::namedWindow(WINDOW2);

  this->binary_threshold = 100;
  this->canny_threshold = 100;
  this->canny_max_thresh = 255;

  // [init publishers]
  this->point_list_publisher_ = this->public_node_handle_.advertise<iri_clothes_grasping_point::GraspingPointList>("point_list", 5);
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

void IriClothesGraspingPointAlgNode::binaryObjectSelection(const cv::Mat labelled_image, cv::Mat& binary_image, unsigned char Red, unsigned char Green, unsigned char Blue)
{

  cvtColor( labelled_image, binary_image, CV_BGR2GRAY );

  if(labelled_image.channels() == 3){
    ROS_INFO("Image depth is 3 assuming RGB");
    for(int i=0;i<binary_image.rows;i++){
      for(int j=0;j<binary_image.cols;j++){
        if(labelled_image.at<Vec3b>(i,j)[0] == Red && labelled_image.at<Vec3b>(i,j)[1] == Green && labelled_image.at<Vec3b>(i,j)[2] == Blue){
          binary_image.at<uchar>(i,j) = 255;
        }else{
//          ROS_INFO(" %d %d %d ", labelled_image.at<Vec3b>(i,j)[0],labelled_image.at<Vec3b>(i,j)[1],labelled_image.at<Vec3b>(i,j)[2]);   
          binary_image.at<uchar>(i,j) = 100;
        }
      } 
    } 
  }else{
    ROS_ERROR("Incorrect number of channels"); 
  }
  
}

void IriClothesGraspingPointAlgNode::draw_contours_center(cv::Mat src_img)
{
  RNG rng(12345);

  cv::Mat src_gray, src_binary;
  cv::Mat canny_output;
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;

  RGBValue color;
//  color.Red = 0; 
//  color.Green = 0; 
//  color.Blue = 0; 
//  color.Red = 184; 
//  color.Green = 201; 
//  color.Blue = 204; 
  color.Red = 0; 
  color.Green = 0; 
  color.Blue = 128; 

  cv::imshow(WINDOW, src_img);
  /// Select one object
  ROS_INFO("Select object");
  this->binaryObjectSelection(src_img, src_binary, color.Red, color.Green, color.Blue);
  ROS_INFO("Show binary");
  cv::imshow(WINDOW2, src_binary);

/*
  ROS_INFO("Grayscale");
  /// Transform to grayscale
//  cvtColor( src_img, src_gray, CV_BGR2GRAY );
//  blur( src_gray, src_gray, Size(3,3) );

  src_binary.copyTo(src_gray);

  ROS_INFO("Canny");
  /// Detect edges using canny
  Canny( src_gray, canny_output, this->canny_threshold, this->canny_threshold*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  ROS_INFO("Moments");
  /// Get the moments
  vector<Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mu[i] = moments( contours[i], false ); }

  ///  Get the mass centers:
  vector<Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
       circle( drawing, mc[i], 4, color, -1, 8, 0 );
     }

  ROS_INFO("Show");
  /// Show in a window
  imshow(WINDOW, drawing );

  /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
  printf("\t Info: Area and Contour Length \n");
  for( int i = 0; i< contours.size(); i++ )
     {
       printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
       circle( drawing, mc[i], 4, color, -1, 8, 0 );
     }
*/
}

void IriClothesGraspingPointAlgNode::count_labels(cv::Mat src_img, std::set<float>& labels){

//TODO getparam to prevent nodes counting over and over again?

  //RGBValue color;
  //check channels
  if(src_img.depth() != 3)
    ROS_ERROR("Incorrect image depth");

  labels.clear();
//  for(int i=0;i<src_img.size().width;i++){
//    for(int j=0;j<src_img.size().height;j++){
//      color.Red = src_img.at<Vec3b>(i,j)[0];
//      color.Green = src_img.at<Vec3b>(i,j)[1];
//      color.Blue = src_img.at<Vec3b>(i,j)[2];
//
//      labels.insert(color.float_value); 
//    } 
//  } 
}

void IriClothesGraspingPointAlgNode::draw_intersections(cv::Mat src_img){

  cv::Mat out_img(src_img);
  cv::Mat str_element = Mat();
  std::set<float> labels;

  count_labels(src_img, labels);
  int numlabels = labels.size();

  for(int i=0;i<numlabels;i++){
    for(int j=i;j<numlabels;j++){
//binary

//dilate
  dilate(src_img, out_img, str_element);

//intersect

//moments
    }
  } 
}

void IriClothesGraspingPointAlgNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat src_gray;
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

  ROS_INFO("Compute moments");
  // Moments for image center of mass calculation
  cvtColor( cv_ptr->image, src_gray, CV_BGR2GRAY );
//  cv::Moments imgmoments = moments(src_gray, true);

  ROS_INFO("Compute and draw moments");
  // Moments drawing
  draw_contours_center(cv_ptr->image);

  ROS_INFO("Intersection heat map");
  // Intersection heat_map
//  draw_intersections(cv_ptr->image);

//  cv::imshow(WINDOW, src_gray);
//  cv::waitKey(3);
  
  image_pub_.publish(cv_ptr->toImageMsg());
  cv::waitKey(10);
}

IriClothesGraspingPointAlgNode::~IriClothesGraspingPointAlgNode(void)
{
  // [free dynamic memory]
    cv::destroyWindow(WINDOW);
    cv::destroyWindow(WINDOW2);
}

void IriClothesGraspingPointAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //this->GraspingPointList_msg.data = my_var;
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
//  this->point_list_publisher_.publish(this->GraspingPointList_msg_);
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void IriClothesGraspingPointAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  ROS_INFO("Reconfigure request : %d %d %d %d %s %d %s %f %d",
           (int)config.a,(int)config.intensity, config.cluster, config.skip, 
           config.port.c_str(),(int)config.calibrate_time, 
           config.frame_id.c_str(), config.time_offset,   
           (int)config.allow_unsafe_settings);
  
  // do nothing for now
    a = config.a;

  this->alg_.unlock();
}

void IriClothesGraspingPointAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<IriClothesGraspingPointAlgNode>(argc, argv, "clothes_grasping_point");
}
