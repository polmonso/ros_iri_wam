#include "zyonz_tof_color_driver_node.h"
#include <string>
#include <iostream>

//to save the image_callback
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <boost/format.hpp>

ZyonzTofColorDriverNode::ZyonzTofColorDriverNode(ros::NodeHandle &nh) : iri_base_driver::IriBaseNodeDriver<ZyonzTofColorDriver>(nh),    
depth_sent_ (true), rgb_sent_ (true), intens_sent_ (true), zbuffer_threshold_(20), saveFile (false), saveFileCounter (0)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  /*
  std::string imageTopic;
  this->private_node_handle_.param<std::string>("image", imageTopic, "a");
  ROS_INFO("Image initializad at %d %s",imageTopic.length(),imageTopic.c_str());
  */
  /*
  if (ros::param::has("image"))
  {
    ros::param::get("image", image);
    ROS_INFO("Image initializad at %s",image.c_str());
  }
  */
  /*
  nh.getParam("/image", imageTopic);
  ROS_INFO("Image initializad at %d %s",imageTopic.length(),imageTopic.c_str());
  */
  
  //matrius de calibratge
  //TODO: llegir-les d'un fitxer
//  K_<<822.24782,0,313.08856,0,822.34043,255.75672,0,0,1;
//  K_<<1618.29, 0, 625.218,0 ,1618.29, 502.417,0,0,1; //NOVA
//CAMCUBE_AQUESTA K_<< 1580,0,604.43,0,1585.13,519.16,0,0,1;//NOVA2
  //passa punts de la tof a la RGB
  //Aquesta és l'original, on hi havia una rotacio per culpa dels SR de les cameres
//  P_<<-0.99995,-0.00912,-0.000585,-1.61384,0.00913,-0.99983,-0.01564,55.924639,-0.00044,-0.01564,0.99987,33.97095,0,0,0,1;
//  P_<<0.999739, 0.0227623, 0.00193176, -7.13658, -0.0226342, 0.998442, -0.0510053, 66.2559, -0.00308975, 0.0509483, 0.998697,30.4039,0,0,0,1; //NOVA
//  P_<<0.999739, 0.0227623, 0.00193176, -7.13658, -0.0226342, 0.998442, -0.0510053, 73.2559, -0.00308975, 0.0509483, 0.998697,36.4039,0,0,0,1; //NOVA
//CAMCUBE_AQUESTA P_ << 1, 0, 0, -0.3921, 0, 1, 0, 59.2096, 0, 0, 1,20.8921,0,0,0,1;//NOVA2
//  P_<<1, 0, 0, -7.13658, 0, 1, 0, 66.2559, 0, 0, 1,30.4039,0,0,0,1; 

//  P_<<-0.99995,-0.00912,-0.000585,-0.00161384,0.00913,-0.99983,-0.01564,0.055924639,-0.00044,-0.01564,0.99987,0.03397095,0,0,0,1;
  kernel_size_=3;
  
  //CAMBOARD + rgb640x480
  
  K_<< 791.241, 0, 323.613, 0, 791.241,266.511,0,0,1;  
  P_ << 0.987, -0.028, -0.156, 54.0775, 0.0322, 0.999, 0.0209, 2.0674, 0.156, -0.0257, 0.988,18.3983,0,0,0,1;
  // [init publishers]
  this->cloud2_raw_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("cloud2_raw", 10);
  
  // [init subscribers]
  this->input_intens_subscriber_ = this->public_node_handle_.subscribe("/input_intens", 5, &ZyonzTofColorDriverNode::input_intens_callback, this);
  //TODO s'ha de fer servir el ImageTransport!!
  //image_transport::ImageTransport it(nh);

  this->image_subscriber_ = this->public_node_handle_.subscribe("/image", 10, &ZyonzTofColorDriverNode::image_callback, this);
  this->pointCloud_subscriber_ = this->public_node_handle_.subscribe("/pointCloud", 10, &ZyonzTofColorDriverNode::pointCloud_callback, this);
  
  // [init services]
  this->saveImage_server_ = this->public_node_handle_.advertiseService("saveImage", &ZyonzTofColorDriverNode::saveImageCallback, this);
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
  
  
  
}

void ZyonzTofColorDriverNode::mainNodeThread(void)
{
  //lock access to driver if necessary
  this->driver_.lock();

  // [fill msg Header if necessary]
  //<publisher_name>.header.stamp = ros::Time::now();
  //<publisher_name>.header.frame_id = "<publisher_topic_name>";

  // [fill msg structures]
  //this->PointCloud2_msg.data = my_var;
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
//  this->cloud2_raw_publisher_.publish(this->PointCloud2_msg_);

  //unlock access to driver if previously blocked
  this->driver_.unlock();  
}

/*  [subscriber callbacks] */
void ZyonzTofColorDriverNode::input_intens_callback(const sensor_msgs::Image::ConstPtr& msg) 
{ 
//  ROS_INFO("ZyonzTofColorDriverNode::input_intens_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->driver_.lock(); 
  //this->input_intens_mutex_.enter(); 

  //std::cout << msg->data << std::endl; 

  //unlock previously blocked shared variables 
  //this->driver_.unlock(); 
  //this->input_intens_mutex_.exit();   
  
  intens_image_ = msg;
  intens_sent_ = false;
  processRgbAndDepth ();
}
void ZyonzTofColorDriverNode::image_callback(const sensor_msgs::Image::ConstPtr& msg) 
{ 
  //lock member message variable 
  //this->image_mutex_.enter(); 

  //lock access to driver if necessary 
  //this->driver_.lock(); 

  //std::cout << msg->data << std::endl; 

  //unlock access to driver if previously blocked 
  //this->driver_.unlock(); 

  //unlock member message variable 
  //this->image_mutex_.exit(); 
  rgb_image_ = msg;
  rgb_sent_ = false;
  processRgbAndDepth ();
}
void ZyonzTofColorDriverNode::pointCloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) 
{ 
  //lock member message variable 
  //this->pointCloud_mutex_.enter(); 

  //lock access to driver if necessary 
  //this->driver_.lock(); 

  //std::cout << msg->data << std::endl; 

  //unlock access to driver if previously blocked 
  //this->driver_.unlock(); 

  //unlock member message variable 
  //this->pointCloud_mutex_.exit(); 
  cloud2_ = msg;
  depth_sent_ = false;
  processRgbAndDepth ();

}


void ZyonzTofColorDriverNode::processRgbAndDepth() {
  if ((!rgb_sent_) && (!depth_sent_) && (!intens_sent_)){
    //  MatrixXf Z_buffer_=MatrixXf::Zero(cloud2_->width,cloud2_->height);
    MatrixXf Z_buffer_=MatrixXf::Zero(rgb_image_->width,rgb_image_->height);
    
    //build the message if someone is listening
    if (cloud2_raw_publisher_.getNumSubscribers () > 0) 
    {
      ROS_DEBUG("Image Size %d %d %d",rgb_image_->width,rgb_image_->height,rgb_image_->step);
      ROS_DEBUG("Cloud Size %d %d",cloud2_->width,cloud2_->height);
      
      //initialize a pointcloud
      //TODO: do it in the constructor?
      //TODO: maybe we can reuse the same message instead of making a copy?
      PointCloud2_msg_.height         = cloud2_->height;
      PointCloud2_msg_.width          = cloud2_->width;
      PointCloud2_msg_.fields.resize (4);
      PointCloud2_msg_.fields[0].name = "x";
      PointCloud2_msg_.fields[1].name = "y";
      PointCloud2_msg_.fields[2].name = "z";
      PointCloud2_msg_.fields[3].name = "rgb";
      PointCloud2_msg_.header.stamp   = cloud2_->header.stamp;
      PointCloud2_msg_.header.frame_id = std::string ("/color_camera_frame");//cloud2_->header.frame_id;
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
      
      //build the Z_buffer
      //TODO: fer una funció que projecti i no replicar el codi!!
      for (uint8_t i=0+kernel_size_;i<cloud2_->width-kernel_size_;i++)
      {
        for (uint8_t j=0+kernel_size_;j<cloud2_->height-kernel_size_;j++)
        {
          Vector4f point3D;
          int index2 = (i)*cloud2_->width +(j);
          float *pstep2 = (float*)&cloud2_->data[(index2) * cloud2_->point_step];
          //la matriu de calibratge esta en mm....
          
          point3D(0)=pstep2[0]*1000;
          point3D(1)=pstep2[1]*1000;
          point3D(2)=pstep2[2]*1000;
          point3D(3)=1.0;
          //std::cout << "point3D\n" << point3D<<std::endl;
          
          // put depth point onto rgb frame
          Vector4f tmp=P_*point3D;        
          // project depth point on the camera
          MatrixXf Kextended(3,4);
          //std::cout << "Kextended\n" << Kextended<<std::endl; 
          Kextended<<K_,Vector3f::Zero(3);
          
          Vector3f projectedPoint=Kextended*tmp;
          //std::cout << "projectedPoint\n" << projectedPoint<<std::endl; 
          
          //normalize
          int u=(int)(projectedPoint(0)/projectedPoint(2));
          int v=(int)(projectedPoint(1)/projectedPoint(2));
          //CAL CONSTRUIR PRIMER EL ZBUFFER
          //SI no quan descobrim un punt mal estiquetat de color no es pot tornar enrere!
          // El Z_buffer conté les profunditats dels punts en SR_rgb. Ha de tenir mida rgb ja que cal projectar els punts!
          //	if ((Z_buffer_(i,j)==0.0) ||(tmp(2)<Z_buffer_(i,j))) 
          if ((u>kernel_size_) && (u < (int)rgb_image_->width-kernel_size_) && (v>kernel_size_) && (v < (int)rgb_image_->height-kernel_size_)) 
          {
            if ((Z_buffer_(u,v)==0.0) ||(tmp(2)<Z_buffer_(u,v))) 
            {
              //	      Z_buffer_(i,j)=tmp(2);
              //	  Z_buffer_.block(i-kernel_size_,j-kernel_size_,2*kernel_size_+1,2*kernel_size_+1)=  MatrixXf::Constant(2*kernel_size_+1,2*kernel_size_+1, tmp(2));
              Z_buffer_.block(u-kernel_size_,v-kernel_size_,2*kernel_size_+1,2*kernel_size_+1)=  MatrixXf::Constant(2*kernel_size_+1,2*kernel_size_+1, tmp(2));
            }
          }
        }
      }
      
      //build the colored pointcloud
      for (uint i=0+kernel_size_;i<cloud2_->width-kernel_size_;i++)
      {
        for (uint j=0+kernel_size_;j<cloud2_->height-kernel_size_;j++)
        {
          Vector4f point3D;
          //int index2 = (i)*cloud2_->width +(j);
          int index2 = (j)*cloud2_->width +(i);
          float *pstep = (float*)&PointCloud2_msg_.data[(index2) * PointCloud2_msg_.point_step];
          float *pstep2 = (float*)&cloud2_->data[(index2) * cloud2_->point_step];
          
          //ATENTION!!!! Temporally save the original points
          if (saveFile) 
            outFile<<pstep2[0]<<" "<<pstep2[1]<<" "<<pstep2[2]<<" "<<0<<" "<<0<<" ";
          
          //if ((i==100)&&(j==100))
          //  std::cout << "point3D" << pstep2[0]<<" "<<pstep2[1]<<" "<<pstep2[2]<<std::endl;
          
          //la matriu de calibratge esta en mm....
            point3D(0)=pstep2[0]*1000;
            point3D(1)=pstep2[1]*1000;
            point3D(2)=pstep2[2]*1000;
            point3D(3)=1.0;
            //std::cout << "point3D\n" << point3D<<std::endl;
            
            // put depth point onto rgb frame
            Vector4f tmp=P_*point3D;	
            //	Vector4f tmp=point3D;	
            
            //copy data in the final pointcloud
            //la matriu de calibratge esta en mm.... pero el point cloud en metres
            pstep[0] =  tmp(0)/1000;
            pstep[1] =  tmp(1)/1000;
            pstep[2] =  tmp(2)/1000;
            //std::cout << "tmp\n" << tmp<<std::endl;        
            // project depth point on the camera
            MatrixXf Kextended(3,4);
            //std::cout << "Kextended\n" << Kextended<<std::endl; 
            Kextended<<K_,Vector3f::Zero(3);
            
            Vector3f projectedPoint=Kextended*tmp;
            //std::cout << "projectedPoint\n" << projectedPoint<<std::endl; 
            
            //normalize
            int u=(int)(projectedPoint(0)/projectedPoint(2));
            int v=(int)(projectedPoint(1)/projectedPoint(2));
            // if (saveFile) 
            //   outFile<<pstep[0]<<" "<<pstep[1]<<" "<<pstep[2]<<" "<<u<<" "<<v<<" ";
            //std::cout << "u,v\n" << u<<" "<<v<<std::endl;
            if ((u>kernel_size_) && (u < (int)rgb_image_->width-kernel_size_) && (v>kernel_size_) && (v<(int)rgb_image_->height-kernel_size_)) 
            {
              //check the Z_buffer
              //	    if (((tmp(2)-zbuffer_threshold_)<Z_buffer_(i,j)) && (Z_buffer_(i,j) !=0 ))
              if (((tmp(2)-zbuffer_threshold_)<Z_buffer_(u,v)) && (Z_buffer_(u,v) !=0 ))
              {
                int rgb_index = v*rgb_image_->step +u*3;
                // 	      int8_t r = rgb_image_->data[(rgb_index)];//imagestep[0];
                // 	      int8_t g = rgb_image_->data[(rgb_index)+1];//imagestep[1];
                // 	      int8_t b = rgb_image_->data[(rgb_index)+2];//imagestep[2];
                // 	      // Fill in RGB
                int32_t rgb_packed = 0;
                // 	      rgb_packed = (r << 16) | (g << 8) | b;
                //rgb_packed = ((int8_t)rgb_image_->data[(rgb_index)] << 16) | ((int8_t)rgb_image_->data[(rgb_index)+1] <<8 ) | (int8_t)rgb_image_->data[(rgb_index)+2];
                rgb_packed = (rgb_image_->data[(rgb_index)] << 16) | (rgb_image_->data[(rgb_index)+1] <<8 ) | rgb_image_->data[(rgb_index)+2];
                memcpy(&pstep[3], &rgb_packed, sizeof(int32_t));
                if (saveFile) 
                  outFile<<"0"<<std::endl;
              } else {
                //std::cout<<"ZBUFFER "<<Z_buffer_(i,j)<<" tmp " << tmp(2) <<std::endl;
                pstep[3] =  (0xff << 16) | (0x00 << 8) | 0x00; //red
                if (saveFile) 
                  outFile<<"1"<<std::endl;
              }
            } else {
              int32_t rgb_packed = 0;
              memcpy(&pstep[3], &rgb_packed, sizeof(int32_t));
              //TODO: this is really out fo range!!!
              if (saveFile) outFile<<"0"<<std::endl;
            }
        }
      }   
      
      this->cloud2_raw_publisher_.publish(this->PointCloud2_msg_);
      
      
    }
    
    
    
    rgb_sent_   = true;
    depth_sent_ = true;
    intens_sent_ = true;
    
    if (saveFile) {
      outFile.close();
      sensor_msgs::CvBridge g_bridge;
      boost::format g_format;  
      g_format.parse("outRGB%04i.%s");
      if (g_bridge.fromImage(*rgb_image_, "bgr8")) {
        IplImage *image = g_bridge.toIpl();
        if (image) {
          std::string filename = (g_format % saveFileCounter % "png").str();
          cvSaveImage(filename.c_str(), image);
          ROS_INFO("Saved image %s", filename.c_str());
          
        } else {
          ROS_WARN("Couldn't save image, no data!");
        }
      } else {
        ROS_ERROR("Unable to convert %s image to bgr8", rgb_image_->encoding.c_str());
      }
      
      //and the intensity    
      g_format.parse("outINTENS%04i.%s");
      
      if (g_bridge.fromImage(*intens_image_, "bgr8")) {
        IplImage *image = g_bridge.toIpl();
        if (image) {
          std::string filename = (g_format % saveFileCounter % "png").str();
          cvSaveImage(filename.c_str(), image);
          ROS_INFO("Saved image %s", filename.c_str());
        } else {
          ROS_WARN("Couldn't save image, no data!");
        }
      }
      else {
        ROS_ERROR("Unable to convert %s image to bgr8", rgb_image_->encoding.c_str());
      }      
      
      saveFile = false; 
    }
  }
}
/*  [service callbacks] */
bool ZyonzTofColorDriverNode::saveImageCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) 
{ 
  //TODO: save the temporal variables and save monolitically here!!!
  //Take care with sichronization: both images should be the really used for computations!

  saveFileCounter++; //TODO: controlar el desbordament!!
  fileName="outCamCube";
  std::stringstream ss;
  ss << saveFileCounter;
  fileName+=ss.str(); // a vegades c++ no posa les coses f?cils!!!!
  fileName+=".m";
  //prepara el fitxer de profunditats
  outFile.open(fileName.c_str());   
  if( !outFile ) {
    ROS_INFO("ZyonzTofColorDriverNode::saveImageCallback: Error obrint el fitxer de sortida!");
    saveFile=false;
  } else {
    outFile<<"% X Y Z u v occlusion "<<std::endl;
    saveFile=true;
  }

  ROS_INFO("ZyonzTofColorDriverNode::saveImageCallback: New Request Received! %s", fileName.c_str()); 

//use appropiate mutex to shared variables if necessary 
  //this->driver_.lock(); 
  //this->saveImage_mutex_.enter(); 

  
  
  //if(this->driver_.isRunning()) 
  //{ 
    //ROS_INFO("ZyonzTofColorDriverNode::saveImageCallback: Processin New Request!"); 
    //do operations with req and output on res 
    //res.data2 = req.data1 + my_var; 
  //} 
  //else 
  //{ 
    //ROS_INFO("ZyonzTofColorDriverNode::saveImageCallback: ERROR: driver is not on run mode yet."); 
  //} 

  //unlock previously blocked shared variables 
  //this->driver_.unlock(); 
  //this->saveImage_mutex_.exit(); 

  return true; 
}

/*  [action callbacks] */

/*  [action requests] */

void ZyonzTofColorDriverNode::postNodeOpenHook(void)
{
}

void ZyonzTofColorDriverNode::addNodeDiagnostics(void)
{
}

void ZyonzTofColorDriverNode::addNodeOpenedTests(void)
{
}

void ZyonzTofColorDriverNode::addNodeStoppedTests(void)
{
}

void ZyonzTofColorDriverNode::addNodeRunningTests(void)
{
}

void ZyonzTofColorDriverNode::reconfigureNodeHook(int level)
{
}

ZyonzTofColorDriverNode::~ZyonzTofColorDriverNode()
{
  // [free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<ZyonzTofColorDriverNode>(argc,argv,"zyonz_tof_color_driver");
}
