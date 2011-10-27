#include "bounding_box.h"

using namespace Eigen;

// [Constructor]
FilterBB::FilterBB() {

  //init class attributes if necessary
  
  // Bounding Box Parameters
  if (!this->nh.getParam("/bounding_box/x_s", this->x_s)){
    this->x_s = -0.5;
    this->nh.setParam("/bounding_box/x_s", this->x_s);
  }
  if (!this->nh.getParam("/bounding_box/y_s", this->y_s)){
    this->y_s = -0.5;
    this->nh.setParam("/bounding_box/y_s", this->y_s);
  }
  if (!this->nh.getParam("/bounding_box/z_s", this->z_s)){
    this->z_s = 0.1;
    this->nh.setParam("/bounding_box/z_s", this->z_s);
  }
  if (!this->nh.getParam("/bounding_box/x_e", this->x_e)){
    this->x_e = 0.5;
    this->nh.setParam("/bounding_box/x_e", this->x_e);
  }
  if (!this->nh.getParam("/bounding_box/y_e", this->y_e)){
    this->y_e = 0.5;
    this->nh.setParam("/bounding_box/y_e", this->y_e);
  }
  if (!this->nh.getParam("/bounding_box/z_e", this->z_e)){
    this->z_e = 0.3;
    this->nh.setParam("/bounding_box/z_e", this->z_e);
  }


  //string for port names
  std::string pcl2_sub_name;
  std::string pcl2_pub_name;

  // [init subscribers]
  pcl2_sub_name = "input";   // Point Cloud 2
  this->pcl2_sub = this->nh.subscribe<sensor_msgs::PointCloud2>(pcl2_sub_name, 1, &FilterBB::pcl2_sub_callback, this);

  // [init publishers]
  pcl2_pub_name = "/bounding_box/pcl2_bbf";  // Point Cloud 2
  this->pcl2_pub = this->nh.advertise<sensor_msgs::PointCloud2>(pcl2_pub_name, 15);

  // [init services]

  // [init clients]
  
  // [init action servers]
  
  // [init action clients]

  ROS_INFO("starting bounding_box_filter_node with limts = (%f, %f, %f) - (%f, %f, %f)", this->x_s, this->y_s, this->z_s, this->x_e, this->y_e, this->z_e);
  ROS_INFO("Use [ rosparam set /bounding_box/x_s 2.0 ] to change the current X coord start limit");
}

FilterBB::~FilterBB(){
  //  cv::destroyWindow(WINDOW);
}

// [subscriber callbacks]
void FilterBB::pcl2_sub_callback(const sensor_msgs::PointCloud2::ConstPtr& pcl2_msg_) { 
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

  // [BOUNDING BOX FILTER ALGORITHM]

  this->nh.getParam("/bounding_box/x_s", this->x_s);
  this->nh.getParam("/bounding_box/y_s", this->y_s);
  this->nh.getParam("/bounding_box/z_s", this->z_s);
  this->nh.getParam("/bounding_box/x_e", this->x_e);
  this->nh.getParam("/bounding_box/y_e", this->y_e);
  this->nh.getParam("/bounding_box/z_e", this->z_e);

  for (unsigned int rr=0; rr<pcl2_msg_->height; rr++){
    for (unsigned int cc=0; cc<pcl2_msg_->width; cc++){
      int idx0 = rr*pcl2_msg_->width + cc;
      float *pstep = (float*)&pcl2_msg_->data[idx0 * pcl2_msg_->point_step];
      float *nstep = (float*)&PointCloud2_msg_.data[idx0 * PointCloud2_msg_.point_step];

      float x_ = nstep[0] = pstep[0];
      float y_ = nstep[1] = pstep[1];
      float z_ = nstep[2] = pstep[2];
      nstep[3] = pstep[3];
      // Bounding Box Threshold
      // std::cout << std::endl << " z_e: " << this->z_e;
      if ( x_ < this->x_s || y_ < this->y_s || z_ < this->z_s || x_ > this->x_e || y_ > this->y_e || z_ > this->z_e) 
	nstep[0] = nstep[1] = nstep[2] = nstep[3] = NAN;
    }
  }

  // [publish Point Cloud 2 and Depth Image]
  this->pcl2_pub.publish(PointCloud2_msg_);
  //  ROS_INFO("ros_info callback");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bounding_box");
  FilterBB filter_bb;
  ros::spin();
  return 0;
}

