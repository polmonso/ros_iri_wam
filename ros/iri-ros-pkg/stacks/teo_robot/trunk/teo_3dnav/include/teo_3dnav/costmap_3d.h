/*   */

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <vector>
#include <math.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include "pcl/kdtree/kdtree_flann.h" 

class costmap_3d
{
public:
        costmap_3d();
        ~costmap_3d();
        
        void compute_normals(const pcl::PointCloud<pcl::PointXYZ>& point_clouds);
        bool is_free ( int pos); //returns false if obstacle and true if not 

        pcl::PointCloud<pcl::Normal> normals_;
        pcl::PointCloud<pcl::PointNormal> cloud_;
        sensor_msgs::PointCloud cloud_t_;    //transofmed cloud to local frame
        double max_height;
        double kneighbours_;
        double radius_;
        bool use_kneighbours_;

protected:
        
        pcl::PointCloud<pcl::PointXYZ> point_clouds_;

};

costmap_3d::costmap_3d() {
   //max_height=0.1;
   ros::param::param<double>("/clouder/max_height_", max_height, 0.08);
  }

costmap_3d::~costmap_3d(){  }


void costmap_3d::compute_normals(const pcl::PointCloud<pcl::PointXYZ>& point_clouds){
    
    point_clouds_=point_clouds;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_ptr_;
    tree_ptr_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
    tree_ptr_->setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (point_clouds));

    std::vector<int> indices;
    for (int i=0;i<(point_clouds.points.size());i++) indices.push_back(i);  

    n.setInputCloud (boost::make_shared <const pcl::PointCloud<pcl::PointXYZ> > (point_clouds));
    n.setIndices (boost::make_shared <std::vector<int> > (indices));
    n.setSearchMethod (tree_ptr_);
    if (use_kneighbours_){
       n.setKSearch (kneighbours_);    // k-neighbours method
    }else{
       n.setRadiusSearch(radius_); // radius method (slower)
    }
    n.compute(normals_);
    return;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool costmap_3d::is_free (int pos){ 

   if (cloud_.points.empty()){
      ROS_DEBUG("Empty Cloud");
      return false;
   }

   if ( fabs(cloud_t_.points[pos].z) < max_height ) { 
      return true; 
   }
   //Possible precipice
   if ( (fabs(cloud_t_.points[pos].z)  >=1) && (fabs(cloud_t_.points[pos].x)<2)  && (fabs(cloud_t_.points[pos].y)<2)) 
   { return false; }



   //BIG Slope
   if (fabs(cloud_.points[pos].normal[2])<0.5){
       ROS_DEBUG("Big slope");
       return false;  
   }

  // Curvature, looking for planes, not isolated points.
  if (cloud_.points[pos].curvature >0.4) {
     return true;
  }
  if (cloud_.points[pos].curvature ==-1){//it means that it was impossible to find enough neighbours when it uses radius search
     return true;     
  }


   //looking for neighbours closer to z=0 
   for (unsigned int i=0;i<(cloud_.points.size());i=i++){ 

       if ( ( fabs(cloud_t_.points[pos].z) < (fabs(cloud_t_.points[pos].z) +0.05) )  //looking for lower points, near in X a Y dimensions. Added 0.05 m  for lecture errors.
                                                     && (fabs(cloud_.points[i].x-cloud_.points[pos].x)<0.15) 
                                                     && (fabs(cloud_.points[i].y-cloud_.points[pos].y)<0.05) ){
          
           if ( (fabs(cloud_.points[i].normal[0] - cloud_.points[pos].normal[0])<0.15) 
             && (fabs(cloud_.points[i].normal[1] - cloud_.points[pos].normal[1])<0.15) 
             && (fabs(cloud_.points[i].normal[2] - cloud_.points[pos].normal[2])<0.15) ) {  //Checking their normals
           //if ( (fabs(normals_.points[i].normal[2] - normals_.points[pos].normal[2])<0.15) ) { 


               if ( (fabs(cloud_.points[i].normal[0])<0.3) && (fabs(cloud_.points[i].normal[1])<0.3) && (fabs(cloud_.points[i].normal[2])<0.99)){                    
                         return true;
               }
           }
       }
   }
   return false;
}



