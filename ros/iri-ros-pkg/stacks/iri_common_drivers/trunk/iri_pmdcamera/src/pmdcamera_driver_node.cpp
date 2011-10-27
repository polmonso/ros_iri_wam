#include "pmdcamera_driver_node.h"
#include <boost/make_shared.hpp>
#include "exceptions.h"

PmdcameraDriverNode::PmdcameraDriverNode(ros::NodeHandle &nh) : iri_base_driver::IriBaseNodeDriver<PmdcameraDriver>(nh) {
  
  max_intensity_ = 50000.0;  // This value doesn't need to be fixed for calibration.(experimentally it didn't overpass 34539.2)
  max_amplitude_ = 50000.0;  // This value needs to be fixed for calibration.(experimentally it didn't overpass 48776.8)
  max_depth_ = 7.5;  // (experimentally it didn't overpass 7.49)
  //TODO: Canviar la max_depth segons la frequencia de modulacio

  //this does not work...
  //pmd_camcube::cameraType camera_type;
  //this->private_node_handle_.param<pmd_camcube::cameraType>("camera_type", camera_type, pmd_camcube::camboard);

  int camera_type=0;
  this->private_node_handle_.param<int>("camera_type", camera_type, pmd_camcube::camboard);

  if (camera_type==pmd_camcube::camboard) {
    this->driver_.setCameraType(pmd_camcube::camboard);
    ROS_INFO("Camera type CAMBOARD");
  } else  if (camera_type==pmd_camcube::camcube) {
    this->driver_.setCameraType(pmd_camcube::camcube);
    ROS_INFO("Camera type CAMCUBE");  
  } else {
    ROS_WARN("Camera type not recognized. Trying camboard...");  
  }
  
  real_width_  = this->driver_.getWidth(); //207;
  real_height_ = this->driver_.getHeight();//204;
  //CamBoard has some dead pixels and efective image size is 200x200
  ROS_INFO(" Image size %d %d", real_width_, real_height_);
  width_  = 200;
  height_ = 200;

  //init class attributes if necessary
  this->loop_rate_ = 50;//in [Hz]

  // Assemble the point cloud data
  if(camera_type==pmd_camcube::camboard){
    PointCloud2_msg_.header.frame_id = std::string ("/camboard_frame");
  } else if (camera_type==pmd_camcube::camcube) {
    PointCloud2_msg_.header.frame_id = std::string ("/camcube3_frame");
  } else {
    ROS_WARN("Camera type not recognized. Trying again...");  
  }

  PointCloud2_msg_.height         = height_;
  PointCloud2_msg_.width          = width_;
  PointCloud2_msg_.fields.resize (4);
  PointCloud2_msg_.fields[0].name = "x";
  PointCloud2_msg_.fields[1].name = "y";
  PointCloud2_msg_.fields[2].name = "z";
  PointCloud2_msg_.fields[3].name = "rgb";

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
  PointCloud2_msg_.is_dense   = true;
  
  // Assemble the intensity image data
  //  Image_int_msg_.header.frame_id = std::string ("/camcube_frame");
  Image_int_msg_.header.frame_id = PointCloud2_msg_.header.frame_id;
  Image_int_msg_.height   = height_;
  Image_int_msg_.width    = width_;
  Image_int_msg_.encoding = "8UC1";
  Image_int_msg_.step     = width_;
  Image_int_msg_.data.resize(width_ * height_);
  
  // Assemble the amplitude image data
  Image_amp_msg_.header.frame_id = PointCloud2_msg_.header.frame_id;
  Image_amp_msg_.height   = height_;
  Image_amp_msg_.width    = width_;
  Image_amp_msg_.encoding = "8UC1";
  Image_amp_msg_.step     = width_;
  Image_amp_msg_.data.resize(width_ * height_);

  // Assemble the depth image data
  Image_depth_msg_.header.frame_id = PointCloud2_msg_.header.frame_id;
  Image_depth_msg_.height   = height_;
  Image_depth_msg_.width    = width_;
  Image_depth_msg_.encoding = "8UC1";
  Image_depth_msg_.step     = width_;
  Image_depth_msg_.data.resize(width_ * height_);


  //@TODO: put the paths in a configuration file?
  //@TODO: podem fer servir public_handler o private_handler 
  // Read calibration parameters from disk
  std::string cam_name, camcube_info_url;
  this->public_node_handle_.param ("camera_name", cam_name, std::string("camera"));
  //  this->public_node_handle_.param ("camcube/camera_info_url", camcube_info_url, std::string("auto"));
  this->public_node_handle_.param ("camera_info_url", camcube_info_url, std::string("auto"));
  if (camcube_info_url.compare("auto") == 0) {
      switch (camera_type) {
	case pmd_camcube::camboard:
	  camcube_info_url = std::string("file://")+ros::package::getPath(ROS_PACKAGE_NAME)+std::string("/info/camboard_calibration.yaml");
	  break;
	case pmd_camcube::camcube:
	  camcube_info_url = std::string("file://")+ros::package::getPath(ROS_PACKAGE_NAME)+std::string("/info/camcube_calibration.yaml");
	  break;
	default:
	  break;
      }
  }

//  camcube_info_manager_ = boost::make_shared<CameraInfoManager> (ros::NodeHandle(public_node_handle_, "camcube"),
//								 cam_name, camcube_info_url);
  camcube_info_manager_ = new CameraInfoManager(public_node_handle_);
  camcube_info_manager_->setCameraName(cam_name);
  if (camcube_info_manager_->validateURL(camcube_info_url))
  {
     camcube_info_manager_->loadCameraInfo(camcube_info_url);
     ROS_INFO ("[CamCubeDriver] Calibration URL:\n\tDepth: %s", camcube_info_url.c_str ());
  }

  camcube_info_ = camcube_info_manager_->getCameraInfo ();
  //  camcube_info_.header.frame_id = std::string ("/camcube_frame"); 
  camcube_info_.header.frame_id = PointCloud2_msg_.header.frame_id; 

  //    ros::NodeHandle param_nh ("~");     // for parameters
  image_transport::ImageTransport imt(this->public_node_handle_);  
  
  // [init publishers]
  this->cloud2_raw_publisher_ = public_node_handle_.advertise<sensor_msgs::PointCloud2>("pointcloud/cloud2_raw", 10);
  //  image_raw_publisher_ = imt.advertiseCamera ("camcube/image_raw", 10);
  //  image_depth_publisher_ = imt.advertiseCamera ("camcube/image_depth", 10);
  image_raw_publisher_ = imt.advertiseCamera ("image_inten", 10); 
  image_amp_publisher_ = imt.advertiseCamera ("image_amp", 10);
  image_depth_publisher_ = imt.advertiseCamera ("image_depth", 10);
  
  ROS_INFO ("publishers initialized");

  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}


void 
PmdcameraDriverNode::findMaxIntensity(const float* intensity)
{
  if (max_intensity_ == 0)
    for (int v = 0; v < height_; ++v)
      for (int u = 0; u < width_; ++u) 
	if (max_intensity_ < intensity[v*(width_)+u])
	  max_intensity_ = intensity[v*(width_)+u];
  //ROS_INFO("max_intensity %d",max_intensity_);
}

void 
PmdcameraDriverNode::findMaxAmplitude(const float* amplitude)
{
  if (max_amplitude_ == 0)
    for (int v = 0; v < height_; ++v)
      for (int u = 0; u < width_; ++u) 
	if (max_amplitude_ < amplitude[v*(width_)+u])
	  max_amplitude_ = amplitude[v*(width_)+u];
  //ROS_INFO("max_amplitude %d",max_amplitude_);
}

void 
PmdcameraDriverNode::findMaxDepth(const float* distance)
{
  if (max_depth_ == 0)
    for (int v = 0; v < height_; ++v)
      for (int u = 0; u < width_; ++u) 
	if (max_depth_ < distance[v*(width_)+u])
	  max_depth_ = distance[v*(width_)+u];
  //ROS_INFO("max_depth %d",max_depth_);
}

void 
PmdcameraDriverNode::assembleIntensityImage(float* intensity)
{
  //we need to obtain the intensity if not done yet
  findMaxIntensity(intensity);

  if (max_intensity_ != 0)
  {
    for (int v = 0; v < height_; ++v)
    {
      for (int u = 0; u < width_; ++u) 
      {
	int index = v * width_ + u;
	//es pot girar la imatge, però és el que cal fer??? 
	//int index2 = u * width_ + v;
	Image_int_msg_.data[index]=intensity[index]*255/max_intensity_;
      }
    }
  }
}

void 
PmdcameraDriverNode::assembleAmplitudeImage(float* amplitude)
{
  //we need to obtain the intensity if not done yet
  //  findMaxAmplitude(amplitude);
  
  if (max_amplitude_ != 0)
  {
    for (int v = 0; v < height_; ++v)
    {
      for (int u = 0; u < width_; ++u) 
      {
	int index = v * width_ + u;
	//es pot girar la imatge, però és el que cal fer??? 
	//int index2 = u * width_ + v;
	Image_amp_msg_.data[index]=amplitude[index]*255/max_amplitude_;   
      }
    }
  }
}

void 
PmdcameraDriverNode::assembleDepthImage(float* distance)
{
  //we need to obtain the intensity if not done yet
  //  max_depth_ = 0; //reset intensity as now we want depth
  //  findMaxDepth(distance);
  
  if (max_depth_ != 0)
  {
    for (int v = 0; v < height_; ++v)
    {
      for (int u = 0; u < width_; ++u) 
      {
	int index = v * width_ + u;
	//es pot girar la imatge, però és el que cal fer??? 
	//int index2 = u * width_ + v;
	Image_depth_msg_.data[index]=distance[index]*255/max_depth_;   
      }
    }
  }
}

void PmdcameraDriverNode::assemblePointCloud2(const float* const coord_3D, const float* const intensity) 
{    
  //we need to obtain the intensity if not done yet
  findMaxIntensity(intensity);
  if (max_intensity_ == 0 ) max_intensity_=1;
 // float bad_point = std::numeric_limits<float>::quiet_NaN ();
    // Assemble an awesome sensor_msgs/PointCloud2 message
    for (int v = 0; v < height_; v++)
    {
      for (int u = 0; u < width_; u++) 
      {
	int index = v * width_ + u;
	int index1 = v * real_width_ + u;
	int index3 = (v * real_width_ + u)*3;

	float *pstep = (float*)&PointCloud2_msg_.data[(index) * PointCloud2_msg_.point_step];
        //flip vertically !!!
        pstep[0] = coord_3D[index3];
        pstep[1] = coord_3D[index3+1];
        pstep[2] = coord_3D[index3+2];
//        if (pstep[2]<0.1)
//	  ROS_WARN("Depth near 0 !!!!!!!!!!!!!!!!!!!!!!!!!!1 %f %d %d",pstep[2], u,v);
        // Fill in RGB
//        unsigned short int ival = intensity[index]*255/max_intensity_;
//	pstep[3] = (0x00<<24) | (ival << 16) | (ival << 8) | ival;
	RGBValue intensity_rgb;
	intensity_rgb.Red = intensity_rgb.Green = intensity_rgb.Blue = intensity[index1]*255/max_intensity_;
        pstep[3] = intensity_rgb.float_value;
/*        uint8_t rgb = intensity[index]*255/max_intensity_;          
	int32_t rgb_packed = 0;
	rgb_packed = (rgb << 16) | (rgb << 8) | rgb;
	memcpy(&pstep[3], &rgb_packed, sizeof(int32_t));
  */    }
    }
}


void PmdcameraDriverNode::mainNodeThread(void)
{
  //reset the intensity max value
  max_intensity_ = 0;
  
  //lock access to driver
  //  this->driver_.lock();
  
  // [fill msg Header if necessary]
  //<publisher_name>.header.stamp = ros::Time::now();
  //<publisher_name>.header.frame_id = "<publisher_topic_name>";
  this->PointCloud_msg_.header.frame_id = PointCloud2_msg_.header.frame_id;
  ros::Time time = ros::Time::now ();
  PointCloud_msg_.header.stamp = PointCloud2_msg_.header.stamp = time;
  Image_int_msg_.header.stamp = camcube_info_.header.stamp = time;
  
  //TODO: blocking the reading part is enough  or all function should be under blockng?
  if ((cloud2_raw_publisher_.getNumSubscribers () > 0) ||
    (image_raw_publisher_.getNumSubscribers () > 0) ||
    (image_amp_publisher_.getNumSubscribers () > 0) ||
    (image_depth_publisher_.getNumSubscribers () > 0)) {
    try {
      this->driver_.lock();
      this->driver_.readData();
      //unlock access to driver if previously blocked
      this->driver_.unlock();
      
      
      // //unlock access to driver if previously blocked
      // this->driver_.unlock();
      
      // [fill msg structures]
      
      // [fill srv structure and make request to the server]
      
      // [fill action structure and make request to the action server]
      
      // [publish messages]
      if (cloud2_raw_publisher_.getNumSubscribers () > 0)
      {
	assemblePointCloud2(this->driver_.get3DImage(),this->driver_.getIntensityImage());
	this->cloud2_raw_publisher_.publish(this->PointCloud2_msg_);
      }
      
      if (image_raw_publisher_.getNumSubscribers () > 0)
      {
	assembleIntensityImage(this->driver_.getIntensityImage());
	//shared to easy the nodelet paradigm
	this->image_raw_publisher_.publish(boost::make_shared<const sensor_msgs::Image> (this->Image_int_msg_),boost::make_shared<const sensor_msgs::CameraInfo> (this->camcube_info_));
      }
      
      if (image_amp_publisher_.getNumSubscribers () > 0)
      {
	assembleAmplitudeImage(this->driver_.getAmplitudeImage());
	//shared to easy the nodelet paradigm
	this->image_amp_publisher_.publish(boost::make_shared<const sensor_msgs::Image> (this->Image_amp_msg_),boost::make_shared<const sensor_msgs::CameraInfo> (this->camcube_info_));
      }
      
      if (image_depth_publisher_.getNumSubscribers () > 0)
      {
	assembleDepthImage(this->driver_.getDepthImage());
	//shared to easy the nodelet paradigm
	this->image_depth_publisher_.publish(boost::make_shared<const sensor_msgs::Image> (this->Image_depth_msg_),boost::make_shared<const sensor_msgs::CameraInfo> (this->camcube_info_));
      }
      //   ROS_INFO("new image");
    } catch(CException &e){
      this->driver_.unlock();
      ROS_INFO("Impossible to capture frame");
    }
    //unlock access to driver if previously blocked
    //  this->driver_.unlock();
  }
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void PmdcameraDriverNode::postNodeOpenHook(void)
{
}

void PmdcameraDriverNode::addNodeDiagnostics(void)
{
}

void PmdcameraDriverNode::addNodeOpenedTests(void)
{
}

void PmdcameraDriverNode::addNodeStoppedTests(void)
{
}

void PmdcameraDriverNode::addNodeRunningTests(void)
{
}

void PmdcameraDriverNode::reconfigureNodeHook(int level)
{
}

PmdcameraDriverNode::~PmdcameraDriverNode()
{
  //[free dynamic memory]
  delete camcube_info_manager_;
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<PmdcameraDriverNode>(argc,argv,"pmdcamera_driver_node");
}
