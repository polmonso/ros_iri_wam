#include "normal_descriptor_alg_node.h"

void init_colormap(std::vector<RGBValue>& colormap)
{
  RGBValue a;
  a.Red=0;    a.Green=0;    a.Blue=128; colormap.push_back(a);
  a.Red=0;    a.Green=0;    a.Blue=144; colormap.push_back(a);
  a.Red=0;    a.Green=0;    a.Blue=160; colormap.push_back(a);
  a.Red=0;    a.Green=0;    a.Blue=176; colormap.push_back(a);
  a.Red=0;    a.Green=0;    a.Blue=192; colormap.push_back(a);
  a.Red=0;    a.Green=0;    a.Blue=208; colormap.push_back(a);
  a.Red=0;    a.Green=0;    a.Blue=225; colormap.push_back(a);
  a.Red=0;    a.Green=0;    a.Blue=241; colormap.push_back(a);
  a.Red=0;    a.Green=2;    a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=18;   a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=34;   a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=51;   a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=67;   a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=83;   a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=99;   a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=115;  a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=132;  a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=148;  a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=164;  a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=180;  a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=196;  a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=212;  a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=229;  a.Blue=255; colormap.push_back(a);
  a.Red=0;    a.Green=245;  a.Blue=255; colormap.push_back(a);
  a.Red=6;    a.Green=255;  a.Blue=249; colormap.push_back(a);
  a.Red=22;   a.Green=255;  a.Blue=233; colormap.push_back(a);
  a.Red=38;   a.Green=255;  a.Blue=217; colormap.push_back(a);
  a.Red=55;   a.Green=255;  a.Blue=200; colormap.push_back(a);
  a.Red=71;   a.Green=255;  a.Blue=184; colormap.push_back(a);
  a.Red=87;   a.Green=255;  a.Blue=168; colormap.push_back(a);
  a.Red=103;  a.Green=255;  a.Blue=152; colormap.push_back(a);
  a.Red=119;  a.Green=255;  a.Blue=136; colormap.push_back(a);
  a.Red=136;  a.Green=255;  a.Blue=119; colormap.push_back(a);
  a.Red=152;  a.Green=255;  a.Blue=103; colormap.push_back(a);
  a.Red=168;  a.Green=255;  a.Blue=87;  colormap.push_back(a);
  a.Red=184;  a.Green=255;  a.Blue=71;  colormap.push_back(a);
  a.Red=200;  a.Green=255;  a.Blue=55;  colormap.push_back(a);
  a.Red=217;  a.Green=255;  a.Blue=38;  colormap.push_back(a);
  a.Red=233;  a.Green=255;  a.Blue=22;  colormap.push_back(a);
  a.Red=249;  a.Green=255;  a.Blue=6;   colormap.push_back(a);
  a.Red=255;  a.Green=245;  a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=229;  a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=213;  a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=196;  a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=180;  a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=164;  a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=148;  a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=132;  a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=115;  a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=99;   a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=83;   a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=67;   a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=51;   a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=34;   a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=18;   a.Blue=0;   colormap.push_back(a);
  a.Red=255;  a.Green=2;    a.Blue=0;   colormap.push_back(a);
  a.Red=241;  a.Green=0;    a.Blue=0;   colormap.push_back(a);
  a.Red=225;  a.Green=0;    a.Blue=0;   colormap.push_back(a);
  a.Red=208;  a.Green=0;    a.Blue=0;   colormap.push_back(a);
  a.Red=192;  a.Green=0;    a.Blue=0;   colormap.push_back(a);
  a.Red=176;  a.Green=0;    a.Blue=0;   colormap.push_back(a);
  a.Red=160;  a.Green=0;    a.Blue=0;   colormap.push_back(a);
  a.Red=144;  a.Green=0;    a.Blue=0;   colormap.push_back(a);
  a.Red=128;  a.Green=0;    a.Blue=0;   colormap.push_back(a);
}


NormalDescriptorAlgNode::NormalDescriptorAlgNode(void)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->descriptor_publisher_ = this->public_node_handle_.advertise<normal_descriptor_node::ndesc_pc>("ndesc", 1); 
  this->heatmap_publisher_ = this->public_node_handle_.advertise<normal_descriptor_node::heat_map>("wrinkled_map", 1);
  this->heatmap_pointcloud_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("wrinkled_point_cloud", 1);
  // [init subscribers]
  this->points_subscriber_ = this->public_node_handle_.subscribe("points", 1, &NormalDescriptorAlgNode::points_callback, this);
  
  // [init services]
  this->wrinkle_map_service_ = this->public_node_handle_.advertiseService("wrinkle_map_service", &NormalDescriptorAlgNode::wrinkle_map_service_callback, this);
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

NormalDescriptorAlgNode::~NormalDescriptorAlgNode(void)
{
  // [free dynamic memory]
}

void NormalDescriptorAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->heatmap_pointcloud_publisher_.publish(this->cloud_out);
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void NormalDescriptorAlgNode::node_config_update(Config &config, uint32_t level)
{
  ROS_INFO("Config update");
}


normal_descriptor_node::heat_map NormalDescriptorAlgNode::compute_wrinkle_map(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
  std::vector<RGBValue> colormap;
  init_colormap(colormap);

  normal_descriptor_node::ndesc_pc ndesc_pc_msg;
  //ndesc_pc_msg = this->alg_.compute_ndescs(cloud);
  ndesc_pc_msg = this->alg_.compute_ndescs_integral(cloud);
  //this->descriptor_publisher_.publish(ndesc_pc_msg);
  
  normal_descriptor_node::heat_map hmap=this->alg_.compute_wrinkled_map(ndesc_pc_msg);
  this->heatmap_publisher_.publish(hmap);

  pcl::PointCloud<pcl::PointXYZRGB> cloud_heat;
  cloud_heat.points.resize(hmap.data.size());
  cloud_heat.width    = cloud.width;
  cloud_heat.height   = cloud.height;
  cloud_heat.is_dense = cloud.is_dense;
  cloud_heat.header   = cloud.header;
  cloud_heat.sensor_origin_ = cloud.sensor_origin_;
  cloud_heat.sensor_orientation_ = cloud.sensor_orientation_;

  double maxim = 0;
  double minim = 999999999;
  double mean = 0;
  for(uint i=0; i<hmap.data.size(); i++)
  {
    if(isnan(hmap.data[i]))
      continue;
    if(hmap.data[i]>maxim)
      maxim = hmap.data[i];
    if(hmap.data[i]<minim)
      minim = hmap.data[i];
    mean += hmap.data[i]; 
    
  }
  mean = mean/hmap.data.size();
  double stdev = 0;
  for(uint i=0; i<hmap.data.size(); i++)
  {
      stdev += (hmap.data[i] - mean)*(hmap.data[i] - mean);
  }
  stdev = sqrt(stdev/hmap.data.size());
  ROS_INFO("Max %f min %f mean %f stdev %f",maxim, minim, mean, stdev ); 
//  maxim = 10;
  minim = 1;
    
  for(uint i=0; i<hmap.data.size(); i++)
  {
    cloud_heat.points[i].x = hmap.points3d[i].x;
    cloud_heat.points[i].y = hmap.points3d[i].y;
    cloud_heat.points[i].z = hmap.points3d[i].z;
    int cmap_idx=(int)(colormap.size() * ((hmap.data[i]-minim)/(maxim-minim)));
    //printf(" %d",cmap_idx);
    cloud_heat.points[i].rgb = colormap[cmap_idx].float_value;
  }
  ROS_INFO("Colormap size: %d",(int)colormap.size());

  pcl::toROSMsg(cloud_heat, this->cloud_out);

  ROS_INFO("wrinkle map published");
  return hmap;
}


void NormalDescriptorAlgNode::points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

  ROS_INFO("Received pointcloud in subscriber");
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*msg, cloud);  
  this->compute_wrinkle_map(cloud);
}


bool NormalDescriptorAlgNode::wrinkle_map_service_callback(normal_descriptor_node::wrinkle::Request &req, normal_descriptor_node::wrinkle::Response &res)
{
  ROS_INFO("disabled for now.");
  return false;
  ROS_INFO("received wrinkle service call");
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(req.cloth_cloud, cloud);
  res.wrinkle_map = this->compute_wrinkle_map(cloud);
  return true; 
}

void NormalDescriptorAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<NormalDescriptorAlgNode>(argc, argv, "normal_descriptor_node");
}

