#include "dummy_mofa_bridge.h"

DummyMofaBridge::DummyMofaBridge() {

  srand((unsigned)time(0)); 

  //init class attributes if necessary
  idx = 0;
  this->nc=0;
  this->nr=0;

  //string for port names
  std::string pcl2_sub_name                    = "/mofa_bridge/input/pcl2/raw";
  std::string labeled_pcl2_sub_name            = "/mofa_bridge/output/pcl2/segmented";
  std::string request_observation_sub_name     = "/mofa_bridge/srv/request_observation";

  // [init subscribers]
  this->pcl2_sub = this->nh.subscribe(pcl2_sub_name, 1, &DummyMofaBridge::pcl2_sub_callback, this);
  
  // [init publishers]
  this->labeled_pcl2_publisher = this->nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> > (labeled_pcl2_sub_name, 5);

  // [init services]
  this->obs_request = this->nh.advertiseService(request_observation_sub_name, &DummyMofaBridge::obs_request_callback, this);

  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
  ROS_INFO("Starting mofaBridge Node");
  ROS_INFO("Use [ rosservice call /request_observation ] to store current range image and compute observation");
}

DummyMofaBridge::~DummyMofaBridge(){

}

void DummyMofaBridge::close(){

}

void DummyMofaBridge::open(){
                
}

/*  [subscriber callbacks] */
void DummyMofaBridge::pcl2_sub_callback(const sensor_msgs::PointCloud2ConstPtr& msg) 
{ 
  pcl::fromROSMsg(*msg, this->pcl_xyzrgb);

  this->nc=msg->width;
  this->nr=msg->height;

}

/*  [service callback] */
bool DummyMofaBridge::obs_request_callback(iri_wam_common_msgs::obs_request::Request &req, iri_wam_common_msgs::obs_request::Response &res){

    /*
  // [xyzRGB matlab file name]
  std::stringstream matlab_xyzrgb_fn;
  //matlab_xyzrgb_fn << "frameXYZRGB" << std::setw(2) << std::setfill('0') << this->idx << ".dat";
  matlab_xyzrgb_fn << "last_image.dat";

  // [open xyzRGB matlab file]
  matlab_xyzrgb.open(matlab_xyzrgb_fn.str().c_str(), std::ios::out);

  RGBValue color;
  // [store xyzRGB matlab file]
  pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = this->pcl_xyzrgb.begin();
  for (int rr = 0; rr < (int)pcl_xyzrgb.height; ++rr) {
    for (int cc = 0; cc < (int)pcl_xyzrgb.width; ++cc, ++pt_iter) {
      pcl::PointXYZRGB &pt = *pt_iter;
      color.float_value= pt.rgb;
      matlab_xyzrgb << rr << " " << cc << " " << pt.x*1000 << " " << pt.y*1000 << " " << pt.z*1000 << " " << (int)color.Red << " " << (int)color.Green << " " << (int)color.Blue << std::endl;
    }
  }

  // [close range image file]
  matlab_xyzrgb.close();
  std::cout << "Saved XYZRGB image " << matlab_xyzrgb_fn.str() << std::endl;

  // DUMMY SAVE
  std::stringstream matlab_xyzrgb_fn_labeled;
  matlab_xyzrgb_fn_labeled << "last_image_labeled.dat";
  matlab_xyzrgb.open(matlab_xyzrgb_fn_labeled.str().c_str(), std::ios::out);

  pt_iter = this->pcl_xyzrgb.begin();
  for (int rr = 0; rr < (int)pcl_xyzrgb.height; ++rr) {
    for (int cc = 0; cc < (int)pcl_xyzrgb.width; ++cc, ++pt_iter) {
      pcl::PointXYZRGB &pt2 = *pt_iter;
      color.float_value= pt2.rgb;
      matlab_xyzrgb << rr << " " << cc << " " << pt2.x*1000 << " " << pt2.y*1000 << " " << pt2.z*1000 << " " << 150 << " " << (int)color.Green << " " << (int)color.Blue << std::endl;
    }
  }

  matlab_xyzrgb.close();
  std::cout << "Saved XYZRGB image " << matlab_xyzrgb_fn_labeled.str() << std::endl;
  reloadRGB();
*/
  res.num_objectsA = rand()%5;
  res.num_objectsB = rand()%2;
  res.num_objects = res.num_objectsA + res.num_objectsB;
  this->labeled_pcl2_publisher.publish(this->pcl_xyzrgb);

  return true;
}

void DummyMofaBridge::reloadRGB(){
    int x, y, z;
    int red_label, green_label, blue_label;
    RGBValue color;
    // [xyzRGB matlab file name]
    std::stringstream matlab_xyzrgb_fn;
    matlab_xyzrgb_fn << "last_image_labelled.dat";

  // [open xyzRGB matlab file]
  matlab_xyzrgb_in.open(matlab_xyzrgb_fn.str().c_str(), std::ios::in);

  // [store xyzRGB matlab file]
  pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = this->pcl_xyzrgb.begin();
  for (int rr = 0; rr < (int)pcl_xyzrgb.height; ++rr) {
    for (int cc = 0; cc < (int)pcl_xyzrgb.width; ++cc, ++pt_iter) {
      pcl::PointXYZRGB &pt = *pt_iter;
      
      matlab_xyzrgb_in >> x;
      matlab_xyzrgb_in >> y;
      matlab_xyzrgb_in >> z;

      matlab_xyzrgb_in >> red_label;
      matlab_xyzrgb_in >> green_label;
      matlab_xyzrgb_in >> blue_label;

      color.Red = red_label;
      color.Green = green_label;
      color.Blue = blue_label;
      pt.rgb = color.float_value;

    }
  }

  // [close range image file]
  matlab_xyzrgb.close();
  std::cout << "Loaded pcl image " << matlab_xyzrgb_fn.str() << std::endl;


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mofa_bridge");
  DummyMofaBridge mofa_bridge;
  mofa_bridge.open();
  ros::spin();
  return 0;
}
