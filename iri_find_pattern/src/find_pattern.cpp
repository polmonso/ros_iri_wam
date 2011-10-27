#include "find_pattern.h"

using namespace Eigen;

FindPattern::FindPattern(ros::NodeHandle &n) : nh_(n), it_(nh_) {

  //string for port names
  image_pub_ = it_.advertise("image_pattern",1);

  //cvNamedWindow("Image window");
  image_sub_ = it_.subscribe("image", 1, &FindPattern::imageCallback, this);

  tf_publisher_ = nh_.advertise<tf::tfMessage>("tf", 5);
  this->tfMessage_msg_.transforms.resize(1); 

  tf_publisher_pattern_ = nh_.advertise<tf::tfMessage>("pattern/tf", 5);
  tf_publisher_pattern_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("pattern/PoseStamped", 5);

  //retrieve the calibration matrices from parameter server
  //camera intrinsic
  std::vector<double> aux_vector;

  if(retrieveMatrixFromParameterServer("Hintrinsic", aux_vector) && aux_vector.size() == 9){
    ROS_INFO("Hintrinsic retrieved from the parameter server");
    for(uint i = 0; i<aux_vector.size(); i++)
      Hintrinsic_data[i] = aux_vector.at(i);
  }

  aux_vector.clear();
  if(retrieveMatrixFromParameterServer("Kdistortion", aux_vector) && aux_vector.size() == 4){
    ROS_INFO("Kdistortion retrieved from the parameter server");
    for(uint i = 0; i<aux_vector.size(); i++)
      Kdistortion_data[i] = aux_vector.at(i);
  }

  aux_vector.clear();
  if(retrieveMatrixFromParameterServer("Hcam2world", aux_vector) && aux_vector.size() == 16){
    ROS_INFO("Hcam2world retrieved from the parameter server");
    for(uint i = 0; i<aux_vector.size(); i++)
      Hcam2world_data[i] = aux_vector.at(i);
  }
  this->cvHcam2world = cvMat(3,3,CV_32FC1,Hcam2world_data);
  btMatrix3x3 aux_mat(Hcam2world_data[0],Hcam2world_data[1],Hcam2world_data[2],
                      Hcam2world_data[4],Hcam2world_data[5],Hcam2world_data[6],
                      Hcam2world_data[8],Hcam2world_data[9],Hcam2world_data[10]);
  float det = aux_mat.determinant();
  aux_mat = btMatrix3x3(Hcam2world_data[0]/det,Hcam2world_data[1]/det,Hcam2world_data[2]/det,
          Hcam2world_data[4]/det,Hcam2world_data[5]/det,Hcam2world_data[6]/det,
          Hcam2world_data[8]/det,Hcam2world_data[9]/det,Hcam2world_data[10]/det);
  tf::Vector3 aux_vec(Hcam2world_data[3], Hcam2world_data[7], Hcam2world_data[11]);
  ROS_INFO("det: %f\n", aux_mat.determinant());
  this->Hcam2world = tf::Transform(aux_mat, aux_vec);

  aux_vector.clear();
  if(retrieveMatrixFromParameterServer("Hgrid2marker", aux_vector) && aux_vector.size() == 16){
    ROS_INFO("Hintrinsic retrieved from the parameter server");
    for(uint i = 0; i<aux_vector.size(); i++)
      Hgrid2marker_data[i] = aux_vector.at(i);
  }
  this->cvHgrid2marker = cvMat(3,3,CV_32FC1,Hgrid2marker_data);
  aux_mat = btMatrix3x3(Hgrid2marker_data[0],Hgrid2marker_data[1],Hgrid2marker_data[2],
                       Hgrid2marker_data[4],Hgrid2marker_data[5],Hgrid2marker_data[6],
                       Hgrid2marker_data[8],Hgrid2marker_data[9],Hgrid2marker_data[10]);
  det = aux_mat.determinant();
  aux_mat = btMatrix3x3(Hgrid2marker_data[0]/det,Hgrid2marker_data[1]/det,Hgrid2marker_data[2]/det,
                       Hgrid2marker_data[4]/det,Hgrid2marker_data[5]/det,Hgrid2marker_data[6]/det,
                       Hgrid2marker_data[8]/det,Hgrid2marker_data[9]/det,Hgrid2marker_data[10]/det);
  tf::Vector3 aux_vec2(Hgrid2marker_data[3], Hgrid2marker_data[7], Hgrid2marker_data[11]);
  ROS_INFO("det: %f\n",aux_mat.determinant());
  this->Hgrid2marker = tf::Transform(aux_mat, aux_vec2);
  
  gray = NULL;
  Hintrinsic = cvMat(3,3,CV_32FC1,Hintrinsic_data);
  Kdistortion = cvMat(4,1,CV_32FC1,Kdistortion_data);

  image_points      = cvCreateMat(BOARDTOTAL,2,CV_32FC1);
  object_points     = cvCreateMat(BOARDTOTAL,3,CV_32FC1);
  intrinsic_matrix  = cvCreateMat(3,3,CV_32FC1);
  distortion_coeffs = cvCreateMat(4,1,CV_32FC1);

  for( int i=0; i<BOARDTOTAL; i++ ) {
      CV_MAT_ELEM(*object_points,float,i,0) = (float) SQUARESIZE*i/BOARD_W;
      CV_MAT_ELEM(*object_points,float,i,1) = (float) SQUARESIZE*(i%BOARD_W);
      CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
  }  
}

FindPattern::~FindPattern()
{
  //cvDestroyWindow("Image window");
}

void FindPattern::mainLoop(){
//handeye published transforms
    tf_br.sendTransform(tf::StampedTransform(tf::Transform(btMatrix3x3(1,0,0,0,1,0,0,0,1), tf::Vector3(0,0,0)), ros::Time::now (),  "openni_rgb_optical_frame","camera")); 
    tf_br.sendTransform(tf::StampedTransform(Hcam2world, ros::Time::now(),   "/wam_fk/wam0", "openni_camera"));
//    tf_br.sendTransform(tf::StampedTransform(Hgrid2marker, ros::Time::now (), "pattern_guess", "wam7"));
    tf_br.sendTransform(tf::StampedTransform(Hgrid2marker, ros::Time::now(), "/wam_fk/wam7", "pattern_guess")); 
}

void FindPattern::imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{

  IplImage *cv_image = NULL;
  std::vector<double> ppose;
  try
  {
    cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("error");
  }

//  if (gray == (IplImage *)NULL || gray->width != cv_image->width || gray->height != cv_image->height)
  gray = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1); // allocate a 1 channel byte image 

  if(findPatternPose(cv_image)){ 
    tf_publisher_.publish(this->tfMessage_msg_);
    tf_publisher_pattern_.publish(this->tfMessage_msg_);
    tf_publisher_pattern_pose_.publish(this->poseStamped_msg_);
  }

  try {
    image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgr8"));
  } catch (sensor_msgs::CvBridgeException error) {
    ROS_ERROR("error");
  }
//  cvShowImage("Image window", cv_image);
//  cvWaitKey(3); 
  cvReleaseImage(&gray);
}

bool FindPattern::findPatternPose(IplImage *cv_image){
  CvPoint2D32f corners[BOARDTOTAL];
  int numCornersFound;
  int found;
  double x,y,z;
  CvMat* rotation_vector = cvCreateMat(3,1,CV_32FC1);
  CvMat* translation_vector = cvCreateMat(3,1,CV_32FC1);
//  cv::Mat rotation_vector = cvCreateMat(3,1,CV_32FC1);
//  cv::Mat translation_vector = cvCreateMat(3,1,CV_32FC1);
  CvMat* rotation_matrix = cvCreateMat(3,3,CV_32FC1);

  // convert color image img to gray image gray:   
  cvCvtColor(cv_image, gray, CV_RGB2GRAY);
  found = cvFindChessboardCorners(gray,cvSize(BOARD_W,BOARD_H),corners,&numCornersFound);
  if(found){

//    ROS_INFO("Found %d\n", numCornersFound);
    cvFindCornerSubPix(gray, corners, numCornersFound, cvSize(5,5), cvSize(-1, -1), cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.1));
    cvDrawChessboardCorners(cv_image,cvSize(BOARD_H,BOARD_W),corners,numCornersFound,found);

    //set camera matrix, 2d array of points, ...
    for( int i=0; i<BOARDTOTAL; i++ ) {
        CV_MAT_ELEM(*image_points, float,i,0) = corners[i].x;
        CV_MAT_ELEM(*image_points, float,i,1) = corners[i].y;
    }

    //Replace with cv::solvePnP??
    cvFindExtrinsicCameraParams2(object_points,image_points,&Hintrinsic,&Kdistortion,rotation_vector,translation_vector);
    //cv::solvePnP(object_points,image_points,&Hintrinsic,&Kdistortion,rotation_vector,translation_vector);
    x = CV_MAT_ELEM(*translation_vector, float, 0,0);
    y = CV_MAT_ELEM(*translation_vector, float, 1,0);
    z = CV_MAT_ELEM(*translation_vector, float, 2,0);

    //transform the rotation vector to rotation matrix
    cvRodrigues2(rotation_vector,rotation_matrix);

//    ROS_INFO("copying results\n");

    //get resultsg
    Matrix3f patternpose(Matrix3f::Identity());

    patternpose << CV_MAT_ELEM(*rotation_matrix, float, 0,0), CV_MAT_ELEM(*rotation_matrix, float, 0,1), CV_MAT_ELEM(*rotation_matrix, float, 0,2),
                   CV_MAT_ELEM(*rotation_matrix, float, 1,0), CV_MAT_ELEM(*rotation_matrix, float, 1,1), CV_MAT_ELEM(*rotation_matrix, float, 1,2),
                   CV_MAT_ELEM(*rotation_matrix, float, 2,0), CV_MAT_ELEM(*rotation_matrix, float, 2,1), CV_MAT_ELEM(*rotation_matrix, float, 2,2);

    Quaternion<float> quat(patternpose);
    this->tfMessage_msg_.transforms[0].header.stamp = ros::Time::now();
    this->tfMessage_msg_.transforms[0].header.frame_id = "camera";
    this->tfMessage_msg_.transforms[0].child_frame_id = "pattern"; 
    this->tfMessage_msg_.transforms[0].transform.translation.x = x/1000.0;
    this->tfMessage_msg_.transforms[0].transform.translation.y = y/1000.0;
    this->tfMessage_msg_.transforms[0].transform.translation.z = z/1000.0;
    this->tfMessage_msg_.transforms[0].transform.rotation.x = quat.x();
    this->tfMessage_msg_.transforms[0].transform.rotation.y = quat.y();
    this->tfMessage_msg_.transforms[0].transform.rotation.z = quat.z();
    this->tfMessage_msg_.transforms[0].transform.rotation.w = quat.w();
  
    this->poseStamped_msg_.header.stamp = ros::Time::now();
    this->poseStamped_msg_.header.frame_id = "camera";
    this->poseStamped_msg_.pose.position.x = x/1000.0;
    this->poseStamped_msg_.pose.position.y = y/1000.0;
    this->poseStamped_msg_.pose.position.z = z/1000.0;
    this->poseStamped_msg_.pose.orientation.x = quat.x();
    this->poseStamped_msg_.pose.orientation.y = quat.y();
    this->poseStamped_msg_.pose.orientation.z = quat.z();
    this->poseStamped_msg_.pose.orientation.w = quat.w();

    ROS_INFO("Pattern detected");
    ROS_INFO("pose: %f %f %f %f %f %f %f",x/1000.0,y/1000.0,z/1000.0,quat.x(),quat.y(),quat.z(),quat.w() );
    //cout << patternpose << endl;
    return true;
  }else{
      ROS_INFO("Pattern not found");
      return false;
  }
}

bool FindPattern::retrieveMatrixFromParameterServer(std::string paramname, std::vector<double> result){

  XmlRpc::XmlRpcValue my_list;
  if(nh_.getParam(paramname, my_list)){
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < my_list.size(); ++i) {
      ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      Hcam2world_data[i] = static_cast<double>(my_list[i]);
    }
    return true;
  }else{
    ROS_WARN("Transformation missing at the parameter server. Using the default matrices.");
    return false;
  } 
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "find_pattern");
  ros::NodeHandle n;
  FindPattern findPattern(n);
  ros::Rate loop_rate(10); 
  while(ros::ok()){
    findPattern.mainLoop();
    ros::spinOnce();
    loop_rate.sleep(); 
  }
  return 0;
}
