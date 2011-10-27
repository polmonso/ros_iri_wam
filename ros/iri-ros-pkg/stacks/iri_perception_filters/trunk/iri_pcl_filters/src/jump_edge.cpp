/*
  Code based on article:
   Fuchs, S. and May, S.
  "Calibration and Registration for Precise
   Surface Reconstruction with TOF Cameras"
 */

#include "jump_edge.h"

using namespace Eigen;

// [Constructor]
FilterJE::FilterJE():it(this->nh) {

  //  cv::namedWindow(WINDOW);

  //init class attributes if necessary
  
  // Camera Intrinsic Parameters
  if (!this->nh.getParam("/jump_edge/sx", this->sx)){
    this->sx = 251.826;
    this->nh.setParam("/jump_edge/sx", this->sx);
  }
  if (!this->nh.getParam("/jump_edge/sy", this->sy)){
    this->sy = 251.913;
    this->nh.setParam("/jump_edge/sy", this->sy);
  }
  if (!this->nh.getParam("/jump_edge/u0", this->u0)){
    this->u0 = 100.55;
    this->nh.setParam("/jump_edge/u0", this->u0);
  }
  if (!this->nh.getParam("/jump_edge/v0", this->v0)){
    this->v0 = 100.88;
    this->nh.setParam("/jump_edge/v0", this->v0);
  }
  if (!this->nh.getParam("/jump_edge/skw", this->skw)){
    this->skw = 0.0;
    this->nh.setParam("/jump_edge/skw",this->skw);
  }
  if (!this->nh.getParam("/jump_edge/ang_thr", this->ang_thr)){
    this->ang_thr = 2.0;
    this->nh.setParam("/jump_edge/ang_thr", this->ang_thr);
  }

  //string for port names
  std::string pcl2_sub_name;
  std::string di_sub_name;
  std::string pcl2_pub_name;
  std::string jei_pub_name;
  std::string jeip_pub_name;

  // [init subscribers]
  pcl2_sub_name = "input";   // Point Cloud 2
  this->pcl2_sub = this->nh.subscribe<sensor_msgs::PointCloud2>(pcl2_sub_name, 1, &FilterJE::pcl2_sub_callback, this);

  // [init publishers]
  jei_pub_name = "/jump_edge/image_depth_jef";   // Depth Image
  this->jei_pub = this->it.advertise(jei_pub_name, 1);

  jeip_pub_name = "/jump_edge/image_depth_pjef";   // Depth Image
  this->jeip_pub = this->it.advertise(jeip_pub_name, 1);

  pcl2_pub_name = "/jump_edge/pcl2_jef";  // Point Cloud 2
  this->pcl2_pub = this->nh.advertise<sensor_msgs::PointCloud2>(pcl2_pub_name, 15);

  // [init services]

  // [init clients]
  
  // [init action servers]
  
  // [init action clients]

  ROS_INFO("starting jump_edge_filter_node with apex_thr = %f", this->ang_thr);
  ROS_INFO("Use [ rosparam set /jump_edge/ang_thr 2.0 ] to change the current angle threshold");
}

FilterJE::~FilterJE(){
  //  cv::destroyWindow(WINDOW);
}

// [subscriber callbacks]
void FilterJE::pcl2_sub_callback(const sensor_msgs::PointCloud2::ConstPtr& pcl2_msg_) { 
  sensor_msgs::PointCloud2 PointCloud2_msg_;

  // Assemble the point cloud data
  
  PointCloud2_msg_.header.frame_id = pcl2_msg_->header.frame_id;
  PointCloud2_msg_.header.stamp    = pcl2_msg_->header.stamp;
  PointCloud2_msg_.height          = pcl2_msg_->height;
  PointCloud2_msg_.width           = pcl2_msg_->width;
  PointCloud2_msg_.fields.resize (4);
  PointCloud2_msg_.fields[0].name  = "x";
  PointCloud2_msg_.fields[1].name  = "y";
  PointCloud2_msg_.fields[2].name  = "z";
  PointCloud2_msg_.fields[3].name  = "intensity";

  // Set all the fields types accordingly
  int offset = 0;
  for (size_t s = 0; s < PointCloud2_msg_.fields.size (); ++s, offset += 4)
  {
    PointCloud2_msg_.fields[s].offset   = offset;
    PointCloud2_msg_.fields[s].count    = 1;
    PointCloud2_msg_.fields[s].datatype = sensor_msgs::PointField::FLOAT32;
  }

  PointCloud2_msg_.point_step = offset;
  PointCloud2_msg_.row_step   = PointCloud2_msg_.point_step * PointCloud2_msg_.width;
  PointCloud2_msg_.data.resize (PointCloud2_msg_.row_step   * PointCloud2_msg_.height);
  PointCloud2_msg_.is_dense = true;

  //Assemble the depth Image
  sensor_msgs::Image Image_msg_;
  Image_msg_.header.frame_id = pcl2_msg_->header.frame_id;
  Image_msg_.header.stamp    = pcl2_msg_->header.stamp;
  Image_msg_.height          = pcl2_msg_->height;
  Image_msg_.width           = pcl2_msg_->width;
  Image_msg_.encoding        = "8UC1";
  Image_msg_.step            = pcl2_msg_->width;
  Image_msg_.data.resize(pcl2_msg_->width * pcl2_msg_->height);

  // [JUMP EDGE FILTER ALGORITHM]

  this->nh.getParam("/jump_edge/sx", this->sx);
  this->nh.getParam("/jump_edge/sy", this->sy);
  this->nh.getParam("/jump_edge/u0", this->u0);
  this->nh.getParam("/jump_edge/v0", this->v0);
  this->nh.getParam("/jump_edge/skw", this->skw);
  this->nh.getParam("/jump_edge/ang_thr", this->ang_thr);

  // focal distance
  float fd = this->sx/(0.5*Image_msg_.width/this->u0);
  // X and Y pixel scale
  float mx = (0.5*Image_msg_.width/this->u0);
  float my = (0.5*Image_msg_.height/this->u0);
  // Angle of incidence in X coords
  float apex_X = atan(mx/fd);
  // Angle of incidence in Y coords
  float apex_Y = atan(my/fd);
  // Angle of incidence in the diagonal XY.
  // apex = 2.0*atan(0.5*n_cc/sx)/n_cc;
  float apex_XY = atan((1/sqrt(pow(mx,2)+pow(my,2)))/fd);

  for (unsigned int rr=0; rr<pcl2_msg_->height; rr++){
    for (unsigned int cc=0; cc<pcl2_msg_->width; cc++){
      int idx0 = rr*pcl2_msg_->width + cc;
      float *pstep = (float*)&pcl2_msg_->data[idx0 * pcl2_msg_->point_step];
      float *nstep = (float*)&PointCloud2_msg_.data[idx0 * PointCloud2_msg_.point_step];

      nstep[0] = pstep[0];
      nstep[1] = pstep[1];
      nstep[2] = pstep[2];
      nstep[3] = pstep[3];

      // Bounding Box Threshold
      // std::cout << std::endl << " z_e: " << this->z_e;
//      if ( x_ < this->x_s || y_ < this->y_s || z_ < this->z_s || x_ > this->x_e || y_ > this->y_e || z_ > this->z_e) 
//	nstep[0] = nstep[1] = nstep[2] = nstep[3] = NAN;

      Image_msg_.data[idx0] = 0.0; // Initialize image to Black

      if (rr==0 || rr == Image_msg_.height-1 || cc == 0 || cc == Image_msg_.width-1 || nstep[2] == 0.0) continue;

      Eigen::Vector3f Va;
      Va(0) = nstep[2]*(cc-this->u0)/this->sx;
      Va(1) = nstep[2]*(rr-this->v0)/this->sy;
      Va(2) = nstep[2];
      int kk=0;
      Eigen::ArrayXf neigh_ang(8);
      for (int ii=-1; ii < 2; ii++){
	for (int jj=-1; jj < 2; jj++){
	  if (ii==0 && jj==0)
	    continue;
	  int tr = rr + ii;
	  int tc = cc + jj;
	  int idx1 = tr*Image_msg_.width + tc;
	  float *pstep1 = (float*)&pcl2_msg_->data[idx1 * pcl2_msg_->point_step];
	  float tmp_value1 = pstep1[2];
	  
	  Eigen::Vector3f Vb;
	  Vb(0) = tmp_value1*(tc-this->u0)/this->sx;
  	  Vb(1) = tmp_value1*(tr-this->v0)/this->sy;
  	  Vb(2) = tmp_value1;
  	  float modVectNeigh = Vb.norm();
  	  float modVectDif = (Va-Vb).norm();
  	  if (ii==jj) {
  	    neigh_ang(kk) = 180.0/M_PI * asin(sin(apex_XY)*(modVectNeigh)/modVectDif);
  	  } else if (ii==0) {
  	    neigh_ang(kk) = 180.0/M_PI * asin(sin(apex_Y)*(modVectNeigh)/modVectDif);
  	  } else if (jj==0) {
  	    neigh_ang(kk) = 180.0/M_PI * asin(sin(apex_X)*(modVectNeigh)/modVectDif);
  	  } else {
  	    neigh_ang(kk) = 180.0/M_PI * asin(sin(apex_XY)*(modVectNeigh)/modVectDif);    
  	  }
  	  kk++;
  	}
      }

      if (neigh_ang.minCoeff() < this->ang_thr){
  	Image_msg_.data[idx0] = 255.0;
	//	nstep[0] = nstep[1] = 
	  nstep[2]=NAN;
      }
      

    }
  }

  // // Convert to OpenCV
  // cv_bridge::CvImagePtr cv_ptr;
  // try{
  //   cv_ptr = cv_bridge::toCvCopy(Image_msg_, "8UC1");
  // }
  // catch (cv_bridge::Exception& e){
  //   ROS_ERROR("cv_bridge exception: %s", e.what());
  //   return;
  // }


  // cv::Mat w_img;
  // cv_ptr->image.copyTo(w_img);
  
  // cv::Size esize(3,3);
  // cv::Mat elem = cv::getStructuringElement(cv::MORPH_CROSS, esize);
  
  // // cv::morphologyEx(w_img, w_img, cv::MORPH_OPEN, elem, cv::Point(-1,-1),1);


  // cv::Size ksize(5,5);
  // cv::blur(w_img, w_img, ksize);
  // cv::blur(cv_ptr->image, cv_ptr->image, ksize);
  // cv::threshold(cv_ptr->image, cv_ptr->image, 50, 255, cv::THRESH_BINARY );  

  
  // cv::threshold(w_img, w_img, 50, 255, cv::THRESH_BINARY );  
  // // cv::dilate(w_img, w_img, elem, cv::Point(-1,-1), 3);
  // // cv::erode(w_img, w_img, elem, cv::Point(-1,-1), 2);

  // // cv::imshow(WINDOW, w_img);
  // // cv::waitKey(3);
  
  // [publish Point Cloud 2 and Depth Image]
  this->pcl2_pub.publish(PointCloud2_msg_);
  this->jei_pub.publish(Image_msg_);  //  this->jei_pub.publish(Image_msg_);
  // this->jeip_pub.publish(cv_ptr->toImageMsg());  // Processed Depth Image
  //  ROS_INFO("ros_info callback");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jump_edge");
  FilterJE filter_je;
  ros::spin();
  return 0;
}

