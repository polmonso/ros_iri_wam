#include "normal_descriptor_alg.h"

//helper fun
inline
int get_bin(float x, float y, std::vector<float> &bin_bounds)
{
  int row=0;
  int col=0;
  int nb=bin_bounds.size();
//  if(x>=bin_bounds[nb-1] || y >=bin_bounds[nb-1])
//     ROS_ERROR("x: %f y: %f",x,y);
  for(int i=0;i<nb;i++)
  {
    if(x<bin_bounds[i] && y<bin_bounds[i])
      break;
    if(x>bin_bounds[i])
      col++;
    if(y>bin_bounds[i])
      row+=nb;
  }
  return row+col;
}

//class definition
NormalDescriptorAlgorithm::NormalDescriptorAlgorithm()
{
  //default values
  this->desc_num_side_spatial_bins_ = 1;
  this->desc_num_orient_bins_ = 24;
  this->desc_patch_radius_=15;  
  this->min_entropy_ = -5;

  //patch radius
  this->set_desc_patch_radius(15);

  //Spatial bins (side)
  this->set_num_side_spatial_bins(1);

  //Orientation bins
  this->set_num_orientation_bins(24);

  //Number of total bins computed automatically

  //Sample each (num) pixels
  this->sample_each_ = 1;
}

NormalDescriptorAlgorithm::~NormalDescriptorAlgorithm()
{
}

void NormalDescriptorAlgorithm::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  if(this->config_.num_spatial_bins != new_cfg.num_spatial_bins)
  {
    this->set_num_side_spatial_bins(new_cfg.num_spatial_bins);
  }
  if(this->config_.num_orientation_bins != new_cfg.num_orientation_bins)
  {
    this->set_num_orientation_bins(new_cfg.num_orientation_bins);
  }
  if(this->config_.desc_patch_radius != new_cfg.desc_patch_radius)
  {
    this->set_desc_patch_radius(new_cfg.desc_patch_radius);
  }
  if(this->config_.sample_each != new_cfg.sample_each)
  {
    this->set_sample_each(new_cfg.sample_each);
  }

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// NormalDescriptorAlgorithm Public API

//normal_descriptor_node::ndesc_pc NormalDescriptorAlgorithm::compute_ndescs(const sensor_msgs::PointCloud2::ConstPtr& msg) 
//{
//  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
//  pcl::PointCloud<pcl::PointXYZRGB> cloud;
//  pcl::fromROSMsg(*msg, cloud);
//  return compute_ndescs(cloud);
//
//}

normal_descriptor_node::ndesc_pc NormalDescriptorAlgorithm::compute_ndescs(pcl::PointCloud<pcl::PointXYZRGB>& pcl_xyzrgb) 
{ 
  ROS_INFO("PointCloud received");

  //TODO check if optimize with voxel-grid or subsampling http://pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid 
  //compute normal
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  pcl::PointCloud<pcl::Normal> pcl_normals;
  normal_estimator.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> > (pcl_xyzrgb));
  normal_estimator.setKSearch (20);
  pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ();
  normal_estimator.setSearchMethod (tree);
  normal_estimator.compute (pcl_normals);

  //get spherical coordinates
  pcl::PointCloud<pcl::PointXY> pcl_spherical; //(ab)using PointXY for holding spherical angles
  pcl_spherical.width  = pcl_xyzrgb.width;
  pcl_spherical.height = pcl_xyzrgb.height;
  pcl_spherical.points.resize (pcl_spherical.width * pcl_spherical.height);
  this->compute_spherical_coords(pcl_normals, pcl_spherical);

  //compute descriptors
  normal_descriptor_node::ndesc_pc ndesc_pc_msg;
  ndesc_pc_msg.len    = this->desc_num_total_bins_;
  ndesc_pc_msg.width  = pcl_spherical.width;
  ndesc_pc_msg.height = pcl_spherical.height;
  
  for(unsigned int u = this->margin_; u < pcl_xyzrgb.width-this->margin_; u += this->sample_each_){
    for(unsigned int v = this->margin_; v < pcl_xyzrgb.height-this->margin_; v += this->sample_each_) 
    {
      normal_descriptor_node::ndesc desc = this->compute_descriptor(pcl_xyzrgb, pcl_spherical, u, v);
      ndesc_pc_msg.descriptors.push_back(desc);
      // perf time for *comp. descriptors* vs *back_pushing* (aka how
      // much would we save with a different implementation). Total
      // time for 300k desc, default param values: 29.95s vs 0.47s

      ndesc_pc_msg.num++;
    }
  }
  ROS_INFO("Computed %d descriptors", ndesc_pc_msg.num);

  return ndesc_pc_msg; //data copying... do with shared pointer?
}

void NormalDescriptorAlgorithm::compute_spherical_coords(pcl::PointCloud<pcl::Normal> &pcl_normal, pcl::PointCloud<pcl::PointXY> &pcl_spherical)
{
  for (size_t i = 0; i < pcl_normal.points.size (); ++i)
  {
    pcl_spherical.points[i].x = acos(pcl_normal.points[i].normal[2]);
    pcl_spherical.points[i].y = atan2(pcl_normal.points[i].normal[1], pcl_normal.points[i].normal[0]);
  }
}


normal_descriptor_node::ndesc NormalDescriptorAlgorithm::compute_descriptor(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<pcl::PointXY> &pcl_spherical, uint u, uint v)
{
  //ROS_INFO("Computing base descriptor");
  normal_descriptor_node::ndesc desc;
  //insert point3D
  desc.point3d.x=cloud(u,v).x;
  desc.point3d.y=cloud(u,v).y;
  desc.point3d.z=cloud(u,v).z;
  
  //insert point2D
  desc.u=u;
  desc.v=v;

  desc.descriptor.resize(this->desc_num_total_bins_);

  uint bound_top = v<this->desc_patch_radius_? 0 : v-this->desc_patch_radius_; 
  uint bound_left = u<this->desc_patch_radius_? 0: u-this->desc_patch_radius_;
  uint bound_right = u-this->desc_patch_radius_ >= cloud.width? cloud.width-1 : u+this->desc_patch_radius_;
  uint bound_bottom = v+this->desc_patch_radius_ >= cloud.height? cloud.height-1 : v+this->desc_patch_radius_;

  for(uint i=bound_left; i<=bound_right; i++){
    for (uint j=bound_top; j<=bound_bottom; j++)
    {
      //filter nans
      if(isnan(pcl_spherical(i,j).x) || isnan(pcl_spherical(i,j).y))
	continue;

      //determine bin
      uint bin=get_bin(pcl_spherical(i,j).x, pcl_spherical(i,j).y, this->orient_bin_bounds_);

      //increment bin  (TODO soft voting)
      desc.descriptor[bin]++;
    }
  }
  //norm histogram/s (L1 norm)
  //TODO: Test L2 norm 
  float total_votes=0;
  for(uint i=0; i<this->desc_num_total_bins_; i++)
    total_votes+=desc.descriptor[i];
  
  if(total_votes != 0){
      for(uint i=0; i<this->desc_num_total_bins_; i++)
        desc.descriptor[i]/=total_votes;
  }

  return desc;
}

normal_descriptor_node::heat_map NormalDescriptorAlgorithm::compute_wrinkled_map(const normal_descriptor_node::ndesc_pc &ndesc_pc_msg)
{
  ROS_INFO("Entering wrinkle map computer.");
  int idx;
  normal_descriptor_node::heat_map hmap;
  hmap.width = ndesc_pc_msg.width;
  hmap.height = ndesc_pc_msg.height;
  hmap.data.resize(hmap.width*hmap.height);
  hmap.points3d.resize(hmap.width*hmap.height);

  //TODO reset vector more efficiently
  for(uint i=0; i<hmap.data.size(); i++)
    hmap.data[i]=0;

  for(uint i=0; i<ndesc_pc_msg.descriptors.size(); i++)
  {
    idx                  = ndesc_pc_msg.descriptors[i].u + ndesc_pc_msg.descriptors[i].v*hmap.width;
    hmap.data[idx]       = this->compute_entropy(ndesc_pc_msg.descriptors[i]);
    hmap.points3d[idx].x = ndesc_pc_msg.descriptors[i].point3d.x;
    hmap.points3d[idx].y = ndesc_pc_msg.descriptors[i].point3d.y;
    hmap.points3d[idx].z = ndesc_pc_msg.descriptors[i].point3d.z;
  }
  ROS_INFO("Exiting wrinkle map computer.");
  
  return hmap; //copy... fer shared pointer
}



////////////////////////////////////////////////////////////////////////////
//// EXPERIMENTAL STUFF ////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

class II
{
public:
  std::vector<int*> ii_;
  uint bins_;
  uint width_;
  uint height_;


  II(uint bins, uint width, uint height):bins_(bins),width_(width),height_(height){
    ii_.resize(bins);
    for(uint i=0;i<bins;i++)
      ii_[i]=new int[width*height];
  }
  ~II(){
    for(uint i=0;i<this->bins_;i++)
      delete [] ii_[i];
  }
  void construct(std::vector<int> &im){
    //0,0
    uint u=0;
    uint v=0;
    for(uint b=0;b<this->bins_;b++)
    {
      //printf("%d %d %d\n",u,v,b);fflush(0);
      ii_[b][this->width_*v+u] =		     \
	(im[this->width_*v+u]==(int)b ? 1:0);
    }

    //n,0
    v=0;
    for(u=1;u<this->width_;u++)
    {
      for(uint b=0;b<this->bins_;b++)
      {
	//printf("%d %d %d\n",u,v,b);fflush(0);
	ii_[b][this->width_*v+u] =		     \
	  ii_[b][this->width_*v+(u-1)] +	     \
	  (im[this->width_*v+u]==(int)b ? 1:0);
	//printf("%d ",im[this->width_*v+u]);
      }
    }
    //printf("0,n\n");fflush(0);
    //0,n
    u=0;
    for(v=1;v<this->height_;v++)
    {
      for(uint b=0;b<this->bins_;b++)
      {
	//printf("%d %d %d\n",u,v,b);fflush(0);
	ii_[b][this->width_*v+u] =		     \
	  ii_[b][this->width_*(v-1)+u] +	     \
	  (im[this->width_*v+u]==(int)b ? 1:0);
	//printf("%d ",im[this->width_*v+u]);
      }
    }
    //n,n
    for(v=1;v<this->height_;v++)
    {
      for(u=1;u<this->width_;u++)
      {
	for(uint b=0;b<this->bins_;b++)
	{
	  //printf("%d %d %d\n",u,v,b);fflush(0);
	  ii_[b][this->width_*v+u] =		     \
	    ii_[b][this->width_*(v-1)+u] +	     \
	    ii_[b][this->width_*v+(u-1)] -	     \
	    ii_[b][this->width_*(v-1)+(u-1)] +	     \
	    (im[this->width_*v+u]==(int)b ? 1:0);
	  //printf("%d ",im[this->width_*v+u]);
	}
      }
    }
    // printf("asdfadsf %d\n",this->bins_);
    // for(uint b=0;b<this->bins_;b++)
    // {
    //   printf("asdfasdfasdfasd\n");
    //   std::string name="/home/aramisa/tmp/debug"+to_string(b)+".txt";
    //   printf("%s\n",name.c_str());
    //   FILE* debug=fopen(name.c_str(),"w");
    //   for(v=0;v<this->height_;v++)
    //   {
    // 	for(u=0;u<this->width_;u++)
    // 	{
    // 	  fprintf(debug,"%d ",ii_[b][this->width_*v+u]);
    // 	}
    // 	fprintf(debug, "\n");
    //   }
    //   fclose(debug);
    // }
  }
  std::vector<int> get_sub_rect(uint tu, uint tv, uint bu, uint bv)
  {
    std::vector<int> ret(this->bins_);
    for(uint b=0;b<this->bins_;b++)
    {
      ret[b] = ii_[b][this->width_*tv+tu] + ii_[b][this->width_*bv+bu] - ii_[b][this->width_*tv+bu] - ii_[b][this->width_*bv+tu];
    }
    return ret;
  }
};





normal_descriptor_node::ndesc_pc NormalDescriptorAlgorithm::compute_ndescs_integral(pcl::PointCloud<pcl::PointXYZRGB>& cloud) 
{

  ROS_INFO("PointCloud received");

  //TODO check if optimize with voxel-grid or subsampling http://pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid 
  //compute normal
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  pcl::PointCloud<pcl::Normal> pcl_normals;
  normal_estimator.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> > (cloud));
  normal_estimator.setKSearch (20);
  pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ();
  normal_estimator.setSearchMethod (tree);
  normal_estimator.compute (pcl_normals);

  //get spherical coordinates
  pcl::PointCloud<pcl::PointXY> pcl_spherical; //(ab)using PointXY for holding spherical angles
  pcl_spherical.width  = cloud.width;
  pcl_spherical.height = cloud.height;
  pcl_spherical.points.resize (pcl_spherical.width * pcl_spherical.height);
  this->compute_spherical_coords(pcl_normals, pcl_spherical);

  //compute descriptors
  normal_descriptor_node::ndesc_pc ndesc_pc_msg;
  ndesc_pc_msg.len    = this->desc_num_total_bins_;
  ndesc_pc_msg.width  = pcl_spherical.width;
  ndesc_pc_msg.height = pcl_spherical.height;
  std::vector<int> binned(cloud.width*cloud.height);

  for(uint v=0;v<cloud.height;v++)
  {
    for(uint u=0;u<cloud.width;u++)
    {
      if(isnan(pcl_spherical(u,v).x) || isnan(pcl_spherical(u,v).y))
	binned[u+cloud.width*v]=-1;
      else
	binned[u+cloud.width*v]=get_bin(pcl_spherical(u,v).x, pcl_spherical(u,v).y, this->orient_bin_bounds_);
    }
  }

  II intim(this->desc_num_orient_bins_*this->desc_num_orient_bins_, cloud.width, cloud.height);
  intim.construct(binned);

  for(uint v=0;v<cloud.height;v++)
  {
    for(uint u=0;u<cloud.width;u++)
    {
      //ROS_INFO("Computing base descriptor");
      normal_descriptor_node::ndesc desc;
      //insert point3D
      desc.point3d.x=cloud(u,v).x;
      desc.point3d.y=cloud(u,v).y;
      desc.point3d.z=cloud(u,v).z;
      
      //insert point2D
      desc.u=u;
      desc.v=v;
      
      desc.descriptor.resize(this->desc_num_total_bins_);

      unsigned int bound_top = v<this->desc_patch_radius_? 0 : v-this->desc_patch_radius_; 
      unsigned int bound_left = u<this->desc_patch_radius_? 0: u-this->desc_patch_radius_;
      unsigned int bound_right = u-this->desc_patch_radius_ >= cloud.width? cloud.width-1 : u+this->desc_patch_radius_;
      unsigned int bound_bottom = v+this->desc_patch_radius_ >= cloud.height? cloud.height-1 : v+this->desc_patch_radius_;
      
      std::vector<int> tmp = intim.get_sub_rect(bound_left, bound_top, bound_right, bound_bottom);
      for(unsigned int i=0;i<this->desc_num_total_bins_;i++)
	desc.descriptor[i]=tmp[i];
      //increment bin  (TODO soft voting)
      //norm histogram/s (L1 norm)
      //TODO: Test L2 norm 
      float total_votes=0;
      for(unsigned int i=0; i<this->desc_num_total_bins_; i++)
	total_votes+=desc.descriptor[i];
  
      if(total_votes != 0){
	for(unsigned int i=0; i<this->desc_num_total_bins_; i++)
	  desc.descriptor[i]/=total_votes;
      }
      ndesc_pc_msg.descriptors.push_back(desc);
      ndesc_pc_msg.num++;
    }
  }
  ROS_INFO("Computed %d descriptors", ndesc_pc_msg.num);

  return ndesc_pc_msg; //data copying... do with shared pointer?
}


normal_descriptor_node::ndesc NormalDescriptorAlgorithm::compute_descriptor_spatial(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<pcl::PointXY> &pcl_spherical, uint u, uint v)
{
  ROS_INFO("Computing spatial descriptor");
  normal_descriptor_node::ndesc desc;
  //insert point3D
  desc.point3d.x = cloud(u,v).x;
  desc.point3d.y = cloud(u,v).y;
  desc.point3d.z = cloud(u,v).z;
  
  //insert point2D
  desc.u = u;
  desc.v = v;

  desc.descriptor.resize(this->desc_num_total_bins_);

  int absolute_top  = u+this->desc_patch_radius_;
  //int absolute_bottom = v+this->desc_patch_radius_;
  int absolute_left = u-this->desc_patch_radius_;
  //int absolute_right = u+this->desc_patch_radius_;
  int size_bin      = (2*this->desc_patch_radius_+1)/this->desc_num_side_spatial_bins_;

  for(uint k=0; k<this->desc_num_side_spatial_bins_; k++)
  {
    uint bound_left  = absolute_left + k*size_bin;
    uint bound_right = absolute_left + (k+1)*size_bin;
    for(uint l=0; l<this->desc_num_side_spatial_bins_; l++)
    {
      uint bound_top    = absolute_top + l*size_bin;
      uint bound_bottom = absolute_top + (l+1)*size_bin;
      for(uint i=bound_left; i<=bound_right; i++)
      {
	for (uint j=bound_top; j<=bound_bottom; j++)
	{
	  //filter nans
	  if(isnan(pcl_spherical(i,j).x) || isnan(pcl_spherical(i,j).y))
	    continue;
	  
	  //determine bin
	  uint offset = (l*this->desc_num_side_spatial_bins_+k) * this->desc_num_orient_bins_;
	  uint bin    = get_bin(pcl_spherical(i,j).x, pcl_spherical(i,j).y, this->orient_bin_bounds_);
	  
	  //increment bin  (TODO soft voting)
	  desc.descriptor[bin+offset]++;
	}
      }
    }
  }
  //norm histogram/s (L1 norm)
  //TODO: Test L2 norm 
  float total_votes=0;
  for(uint i=0; i<this->desc_num_total_bins_; i++)
    total_votes+=desc.descriptor[i];
  
  for(uint i=0; i<this->desc_num_total_bins_; i++)
    desc.descriptor[i]/=total_votes;

  return desc;
}

// normal_descriptor_node::ndesc_pc NormalDescriptorAlgorithm::compute_ndescs_incr1(const sensor_msgs::PointCloud2::ConstPtr& msg) 
// { 
// //  ROS_INFO("PointCloud received");
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
//   pcl::fromROSMsg(*msg, *cloud);

//   //compute normal
//   pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
//   pcl::PointCloud<pcl::Normal> pcl_normals;
//   normal_estimator.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> > (*cloud));
//   normal_estimator.setKSearch (20);
//   pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ();
//   normal_estimator.setSearchMethod (tree);
//   normal_estimator.compute (pcl_normals);

//   //get spherical coordinates
//   pcl::PointCloud<pcl::PointXY> pcl_spherical; //(ab)using PointXY for holding spherical angles
//   pcl_spherical.width  = cloud->width;
//   pcl_spherical.height = cloud->height;
//   pcl_spherical.points.resize (pcl_spherical.width * pcl_spherical.height);
//   this->compute_spherical_coords(pcl_normals, pcl_spherical);

//   //compute descriptors
//   normal_descriptor_node::ndesc_pc ndesc_pc_msg;
//   ndesc_pc_msg.len    = this->desc_num_total_bins_;
//   ndesc_pc_msg.width  = pcl_spherical.width;
//   ndesc_pc_msg.height = pcl_spherical.height;

//   if(this->sample_each_ != 1)
//   {
//     ROS_ERROR("ERROR: Increment should be one for this method");
//   }


//   normal_descriptor_node::ndesc desc;
//   desc.descriptor.resize(this->desc_num_total_bins_);
//   vector< vector<int> > list_votes;

//   for(unsigned int v = this->margin_; v < msg->height-this->margin_; v += 2*this->sample_each_) 
//   {
//     int i=0;
//     //forward
//     for(unsigned int u = this->margin_; u < msg->width-this->margin_; u += this->sample_each_){
//     {
//       float total_votes=this->compute_descriptor(*cloud, pcl_spherical, desc, u, v, list_votes);
//       ndesc_pc_msg.descriptors.push_back(desc);

//       //normalize
//       normal_descriptor_node::ndesc &new_desc=ndesc_pc_msg.descriptors.end();
//       for(i=0; i<this->desc_num_total_bins_; i++)
// 	new_desc->descriptor[i]/=total_votes;
      
//       ndesc_pc_msg.num++;
//     }

//     //update list_votes for new line
//     int bound_top = v-this->desc_patch_radius_ < 0? 0 : v-this->desc_patch_radius_; 
//     int bound_left = u-this->desc_patch_radius_ < 0? 0: u-this->desc_patch_radius_;
//     int bound_right = u-this->desc_patch_radius_ >= cloud.width? cloud.width-1 : u+this->desc_patch_radius_;
//     int bound_bottom = v+this->desc_patch_radius_ >= cloud.height? cloud.height-1 : v+this->desc_patch_radius_;
    
//     i=0;
//     for(list<int>::iterator col=list_votes.begin(); col!=list_votes.end(); col++, i++)
//     {
//       desc.descriptor[col->pop_front()]--;
//       // add new point
//       int bin=get_bin(pcl_spherical(i,bound_bottom).x, pcl_spherical(i,bound_bottom).y, this->orient_bin_bounds_);
//       desc.descriptor[bin]++;
//       col->push_back(bin);
//     }
    
    
//     //backwards
//     for(unsigned int u = msg->width-this->margin_-1; u >= this->margin_; u -= this->sample_each_){
//     {
//       normal_descriptor_node::ndesc desc = this->compute_descriptor(*cloud, pcl_spherical, u, v);
//       ndesc_pc_msg.descriptors.push_back(desc);
//       // perf time for *comp. descriptors* vs *back_pushing* (aka how
//       // much would we save with a different implementation). Total
//       // time for 300k desc, default param values: 29.95s vs 0.47s

//       ndesc_pc_msg.num++;
//     }


//   }
//   ROS_INFO("Computed %d descriptors", ndesc_pc_msg.num);

//   return ndesc_pc_msg; //data copying... do with shared pointer?
// }


// normal_descriptor_node::ndesc NormalDescriptorAlgorithm::compute_descriptor_incr1(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<pcl::PointXY> &pcl_spherical, normal_descriptor_node::ndesc &desc, int u, int v, vector< list<int> > &list_votes)
// {

//   //insert point3D
//   desc.point3d.x=cloud(u,v).x;
//   desc.point3d.y=cloud(u,v).y;
//   desc.point3d.z=cloud(u,v).z;
  
//   //insert point2D
//   desc.u=u;
//   desc.v=v;

//   int bound_top = v-this->desc_patch_radius_ < 0? 0 : v-this->desc_patch_radius_; 
//   int bound_left = u-this->desc_patch_radius_ < 0? 0: u-this->desc_patch_radius_;
//   int bound_right = u-this->desc_patch_radius_ >= cloud.width? cloud.width-1 : u+this->desc_patch_radius_;
//   int bound_bottom = v+this->desc_patch_radius_ >= cloud.height? cloud.height-1 : v+this->desc_patch_radius_;
  
//   int i;
//   if(left_right==1)
//   {
//     i=bound_right;
//   }
//   else
//   {
//     i=bound_left;
//   }
  
  
//   list<int> old_votes = list_votes.pop_front();
//   list<int> new_votes;
  
//   //remove old votes
//   for(list<int>::iterator bin=old_votes.begin(); bin!=old_votes.end(); bin++)
//   {
//     desc.descriptor[*bin]--;
//   }
  
//   //collect new votes
//   for(int j=bound_top; j<=bound_bottom; j++)
//   {
//     int bin=get_bin(pcl_spherical(i,j).x, pcl_spherical(i,j).y, this->orient_bin_bounds_);
//     desc.descriptor[bin]++;
//     new_votes.push_back(bin);
//   }
//   list_votes.push_back(new_votes);
  
// }

