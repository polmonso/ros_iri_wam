#include "filter_table_alg.h"

FilterTableAlgorithm::FilterTableAlgorithm(void)
{
  this->max_z_=0.8;
  this->sac_dist_thr_=0.001;
  this->filter_final_ratio_=0.6;
  this->rad_radius_search_=0.1;
}

FilterTableAlgorithm::~FilterTableAlgorithm(void)
{
}

void FilterTableAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  if(this->config_.max_z != new_cfg.max_z)
  {
    this->set_max_z(new_cfg.max_z);
  }
  if(this->config_.sac_dist_thr != new_cfg.sac_dist_thr)
  {
    this->set_sac_dist_thr(new_cfg.sac_dist_thr);
  }
  if(this->config_.filter_final_ratio != new_cfg.filter_final_ratio)
  {
    this->set_filter_final_ratio(new_cfg.filter_final_ratio);
  }
  if(this->config_.rad_radius_search != new_cfg.rad_radius_search)
  {
    this->set_rad_radius_search(new_cfg.rad_radius_search);
  }


  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// FilterTableAlgorithm Public API
pcl::PointCloud<pcl::PointXYZRGB>::Ptr FilterTableAlgorithm::filter_table(const sensor_msgs::PointCloud2::ConstPtr &cloud_blob)
{
  sensor_msgs::PointCloud2::Ptr cloud_filtered_blob (new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);

  // pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  // sor.setInputCloud (cloud_blob);
  // sor.setLeafSize (0.001f, 0.001f, 0.001f);
  // sor.filter (*cloud_filtered_blob);
  // pcl::fromROSMsg (*cloud_filtered_blob, *cloud_filtered);
  pcl::fromROSMsg (*cloud_blob, *cloud_filtered);

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, this->max_z_);
  pass.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (this->sac_dist_thr_);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 60% of the original cloud is still there
  while (cloud_filtered->points.size () > this->filter_final_ratio_ * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    //filter extracted points by main color
    //0.create bidimensional matrix to vote

    //1.iterate all selected indices and retrieve+convert color + vote in hist
    for(int j=0:j<inliers->indices.size();j++)
    {
      
      
    }
    //2.redo inliers list with only the index in the max bin of the hist




    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_filtered);

    i++;
  }

  //fer radius outlier removal
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> rad;
  rad.setInputCloud(cloud_filtered);
  rad.setRadiusSearch(this->rad_radius_search_);
  rad.setMinNeighborsInRadius(10000);
  rad.filter(*cloud_filtered);
  return cloud_filtered;
}
