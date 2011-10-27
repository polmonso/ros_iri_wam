#include <sensor_msgs/LaserScan.h>
#include <laser_assembler/AssembleScans.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <teo_3dnav/costmap_3d.h>  
#include <pcl/filters/voxel_grid.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include <iri_segway_rmp_msgs/SegwayRMP400Status.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>


using namespace laser_assembler;

class Clouder
{

public:

  sensor_msgs::PointCloud prev_cloud_ [10];
  pcl::PointCloud<pcl::Normal> prev_normals_ [10];
  pcl::PointCloud<pcl::PointNormal> complete_cloud_c;
  const tf::TransformListener& listener;
  int pos_;   //Indicate next position to store
  int prev_;  //indicate how many previous clouds we will use
  int first_times;
  int new_size_;
  bool downsampling; //if point cloud will be downsampled
  double leafsize;   //if downsampling is true, indicates cube size
  bool rotate;       //if pitch status will be used to rotate the point cloud.
  Eigen::Matrix4f rtmatrix_;
  costmap_3d Normalizer;

  Clouder(const tf::TransformListener& listenerin):listener(listenerin)
  {
    pub_cc = n_.advertise<sensor_msgs::PointCloud2> ("complete_assembled_cloud", 10);
    pub_pc = n_.advertise<sensor_msgs::PointCloud> ("parcial_cloud", 10);
    pub_n  = n_.advertise<pcl::PointCloud<pcl::Normal> >("complete_assembled_normals", 10);
    pub_cyn = n_.advertise<pcl::PointCloud<pcl::PointNormal> >("assembled_cloud_with_normals", 10);

    // Create the service client for calling the assembler
    client_c = n_.serviceClient<AssembleScans>("complete_assemble_clouds");
    previous_time=ros::Time::now();
    pos_ =0;
    n_.param<int>("/clouder/prev_clouds_", prev_, 3);

    first_times=0;  
    n_.param<double>("/clouder/leafsize", leafsize, 0.02);
    n_.param<bool>("/clouder/downsampling", downsampling, true);
    n_.param<bool>("/clouder/rotate", rotate, false);    


     n_.param<double>("/clouder/max_height_", Normalizer.max_height, 0.08);
     n_.param<double>("/clouder/kneighbours_", Normalizer.kneighbours_, 20);
     n_.param<bool>("/clouder/use_kneighbours_", Normalizer.use_kneighbours_, true);
     n_.param<double>("/clouder/radius_", Normalizer.radius_, 0.2);

    rtmatrix_.setIdentity();  
             
    
  }

  void NewCloud(const sensor_msgs::PointCloud2 point_cloud)
  {
     AssembleScans srv_clouds;

     //local Vars 
     sensor_msgs::PointCloud cloud_msg_out;
     sensor_msgs::PointCloud aux_cloud;
     sensor_msgs::PointCloud2 point_cloud2;
     sensor_msgs::PointCloud2 point_cloud_sampled;
     pcl::PointCloud<pcl::Normal> normal_msg_out;
     pcl::PointCloud<pcl::PointXYZ> cloud_in;
     pcl::PointCloud<pcl::PointXYZ> cloud_out;

     //DownSampling
     if (downsampling){
        pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
        sor.setInputCloud (boost::make_shared <const sensor_msgs::PointCloud2> (point_cloud) );
        sor.setLeafSize (leafsize, leafsize, leafsize);
        sor.filter (point_cloud_sampled);
        pcl::fromROSMsg(point_cloud_sampled, cloud_in);

        sensor_msgs::convertPointCloud2ToPointCloud(point_cloud_sampled, cloud_msg_out); 	
     }else{
        //conversions
        pcl::fromROSMsg(point_cloud, cloud_in);
        sensor_msgs::convertPointCloud2ToPointCloud(point_cloud, cloud_msg_out); 	
     }
     if (cloud_in.points.empty()){
         ROS_DEBUG("Emty cloud");
         return;
     }

     if((rotate)){
        //Change frame
        listener.transformPointCloud("base_footprint",cloud_msg_out, cloud_msg_out);
        sensor_msgs::convertPointCloudToPointCloud2( cloud_msg_out, point_cloud_sampled); 
        pcl::fromROSMsg(point_cloud_sampled, cloud_in);
        //transform
        if ((fabs(angle_)> 0.025)){ //due to segway sensor noise	
            pcl::transformPointCloud<pcl::PointXYZ>(cloud_in, cloud_in, rtmatrix_);
        }
        //pcl::copyPointCloud<pcl::PointXYZ>(cloud_out, cloud_in);
        pcl::toROSMsg(cloud_in, point_cloud_sampled);
        sensor_msgs::convertPointCloud2ToPointCloud(point_cloud_sampled, cloud_msg_out); 
        //Change to original frame
        listener.transformPointCloud("odom",cloud_msg_out, cloud_msg_out);
        sensor_msgs::convertPointCloudToPointCloud2( cloud_msg_out, point_cloud_sampled); 
        pcl::fromROSMsg(point_cloud_sampled, cloud_in);
     }

     //Compute normals

     Normalizer.compute_normals(cloud_in);

     if (first_times==0){
         normal_msg_out=Normalizer.normals_;
         prev_normals_[pos_]=Normalizer.normals_;
         prev_cloud_[pos_]=cloud_msg_out;
     }

     unsigned int lookback;  //Oldest cloud inside the buffer to enssamble
     if (first_times <= 9){
        if (pos_ <= (prev_-1)) {
           lookback=0;
        }else{
           lookback=pos_-prev_;
        }
     }else{
        if (pos_ <= (prev_-1)) {
           lookback=10+pos_-prev_;
        }else{
           lookback=pos_-prev_;
        }
     }
     if ((lookback >9) || (lookback<0)){
        ROS_DEBUG("Error in lookback %u, pos %u and/or prev %u", lookback, pos_, prev_);
     }
     //////////////////////////////////////////////////////////////////////////////////////////////////////
     //Normal assembler
     normal_msg_out =  prev_normals_[lookback];
     //ROS_DEBUG("lookback %u,  pos %u , prev %u & times %u", lookback, pos_, prev_, first_times);
     if ((first_times < prev_) && (first_times >0)) {
        normal_msg_out = prev_normals_[0];
       for (int i=1; i < (pos_);i++){
          normal_msg_out +=  prev_normals_[i];
        }

      }
     if ((first_times>=prev_) && (first_times<=9)) {
        for (int i=lookback+1; i <= (pos_-1);i++){
           normal_msg_out +=  prev_normals_[i];
        }
     }
     if (first_times>9){
        if (lookback+prev_ >9){
           if (lookback <9){
              for (int i=lookback+1; i <= 9;i++){
                 normal_msg_out +=  prev_normals_[i];
              }
           }
              for (int i=0; i < pos_;i++){
                 normal_msg_out +=  prev_normals_[i];
              }
           
        }else{
           for (int i=lookback+1; i < pos_;i++){
              normal_msg_out +=  prev_normals_[i];
           }
        }
     }

     normal_msg_out += Normalizer.normals_;
     //////////////////////////////////////////////////////////////////////////////////////////////////////
     pub_pc.publish(cloud_msg_out);
     if (first_times==0){
         first_times++;
         pos_++;
         return;
     }
     srv_clouds.request.begin = prev_cloud_[lookback].header.stamp;
     srv_clouds.request.end   = ros::Time::now();
     client_c.call(srv_clouds);

     //ROS_INFO("Time STAMP cloud %u", srv_clouds.response.cloud.header.stamp.toNSec());
     listener.transformPointCloud("floor_laser",srv_clouds.response.cloud, aux_cloud);
     sensor_msgs::convertPointCloudToPointCloud2(aux_cloud, point_cloud2); 
     pcl::fromROSMsg(point_cloud2, cloud_in);
     //Concatenate cloud with normals
     normal_msg_out.header.frame_id=point_cloud2.header.frame_id;
     ROS_DEBUG("Cloud with %u points", (uint32_t)(cloud_in.points.size()));
     ROS_DEBUG("Normals with %u points", (uint32_t)(normal_msg_out.points.size()));
     pcl::concatenateFields (cloud_in, normal_msg_out, complete_cloud_c);		
     

     pub_cc.publish(point_cloud2);
     pub_n.publish(normal_msg_out);
     pub_cyn.publish(complete_cloud_c);

     //Store clouds and normals
     prev_cloud_[pos_]=cloud_msg_out;
     prev_normals_[pos_]=Normalizer.normals_;
     if (pos_ < 9){
         pos_++;
     }else{
         pos_=0;
     }
     if (first_times<500){
        first_times++;
     }
  }

  void NewAngle(iri_segway_rmp_msgs::SegwayRMP400Status SegwayStatus){ 
     double ct = cos((SegwayStatus.rmp200[0].pitch_angle + SegwayStatus.rmp200[1].pitch_angle)/2);
     double st = sin((SegwayStatus.rmp200[0].pitch_angle + SegwayStatus.rmp200[1].pitch_angle)/2);
     angle_=((SegwayStatus.rmp200[0].pitch_angle + SegwayStatus.rmp200[1].pitch_angle)/2);
     rtmatrix_(0,0)= ct;
     rtmatrix_(0,2)= st;
     rtmatrix_(2,0)= -st;
     rtmatrix_(2,2)= ct;

  }



private:
  ros::NodeHandle n_;
  ros::Publisher pub_pc;
  ros::Publisher pub_cc;
  ros::Publisher pub_cyn;
  ros::Publisher pub_n;
  ros::ServiceClient client_c;
  ros::Time previous_time;
  ros::Timer timer_;
  bool first_time_;
  sensor_msgs::PointCloud cloud_floor_laser_;
  double angle_;

} ;



int main(int argc, char **argv)
{
  ros::init(argc, argv, "clouder");
  ros::NodeHandle n;
  tf::TransformListener listener(ros::Duration(10));
  Clouder cloud(listener);
  ros::Subscriber sub =n.subscribe<sensor_msgs::PointCloud2>("snap/assembled_cloud", 10, &Clouder::NewCloud, &cloud);
  ros::Subscriber sub_r =n.subscribe<iri_segway_rmp_msgs::SegwayRMP400Status>("teo/segway/status", 10, &Clouder::NewAngle, &cloud);
  ros::spin();
  return 0;
}
