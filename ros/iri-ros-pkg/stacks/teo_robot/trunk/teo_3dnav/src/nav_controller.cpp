#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/GridCells.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <teo_3dnav/costmap_3d.h>  
#include <pcl/filters/voxel_grid.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include <iri_segway_rmp_msgs/SegwayRMP400Status.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/filters/passthrough.h>


class Nav_Controller
{

public:

  Nav_Controller(const tf::TransformListener& listenerin):listener(listenerin),zoneA_(false),zoneB_(false),zoneC_(false),zoneD_(false),zoneE_(false),zoneF_(false), current_velocity_(true), new_velocity_(true)
  {

    pub_cmd_vel = n_.advertise<geometry_msgs::Twist> ("teo/segway/cmd_vel", 10);
    pub_obstacle_grid = n_.advertise<nav_msgs::GridCells> ("nav_controller/obstacle_grid", 10);
    n_.param<bool>("/nav_controller/enabled", enabled_, true); 
    n_.param<double>("/nav_controller/obstacle_range", obstacle_range_, 4);  //max distance to search for obstacles
    //Zones size
    n_.param<double>("/nav_controller/xmax", xmax_, 2.6);
    n_.param<double>("/nav_controller/xmed", xmed_, 1.6);
    n_.param<double>("/nav_controller/xmin", xmin_, 0.6);
    n_.param<double>("/nav_controller/ymax", ymax_, 1.2);
    n_.param<double>("/nav_controller/ymed1", ymed1_, -0.4);
    n_.param<double>("/nav_controller/xmed2", ymed2_, 0.4);
    n_.param<double>("/nav_controller/ymin", ymin_, -1.2);
    //Max obstacle height
    n_.param<double>("/nav_controller/max_obstacle_height_",max_obstacle_height_, 2);
    //Frequency 
    n_.param<double>("/nav_controller/frequency", frequency_, 10);
    //Velocity contrainst when obstacle appears
    n_.param<double>("/nav_controller/soft_turn", soft_turn_, 0.09);
    n_.param<double>("/nav_controller/tough_turn", tough_turn_, 0.19);
    n_.param<double>("/nav_controller/low_vel", low_vel_, 0.09);
    n_.param<double>("/nav_controller/high_vel", high_vel_, 0.19);
    n_.param<double>("/nav_controller/yaw_tolerance", yaw_tolerance, 0.08);

    //Grid for visualisation purposes
    obstacle_grid_.header.frame_id="/base_footprint";
    obstacle_grid_.cell_height=(xmax_-xmed_);
    obstacle_grid_.cell_width=(ymax_-ymed2_);
    obstacle_grid_.cells.resize(6);


    n_.param<bool>("/clouder/downsampling", downsampling_, true);  
    n_.param<double>("/clouder/max_height_", planes.max_height, 0.08);
    ros::param::param<int>("/clouder/precipice_threshold", precipice_threshold, 300);

    Twistoutmsgs_.linear.x=0;
    Twistoutmsgs_.linear.y=0;
    Twistoutmsgs_.linear.z=0;
    Twistoutmsgs_.angular.x=0;
    Twistoutmsgs_.angular.y=0;
    Twistoutmsgs_.angular.z=0;
    
    timer_ = n_.createTimer(ros::Duration(1/frequency_), &Nav_Controller::TestVelocity, this);

    first_time_ = true;
  }


  void FloorLaserCallback(const pcl::PointCloud<pcl::PointNormal> point_cloud){
      bool zoneA_aux = false;
      bool zoneB_aux = false;
      bool zoneC_aux = false;
      bool zoneD_aux = false;
      bool zoneE_aux = false;
      bool zoneF_aux = false;

      pcl::PointCloud<pcl::PointNormal> cloud =point_cloud;   
                                                                    
      int precipice_count;                                                     // Counting how many points marks as precipice
      signed int precipice_obs;
      double sq_obstacle_range = obstacle_range_ ;
      bool precipice_flag;

      precipice_flag=false;
      //transform point cloud
      sensor_msgs::PointCloud2 aux_cloud_2;
      sensor_msgs::PointCloud aux_cloud1a;
      sensor_msgs::PointCloud aux_cloud1b;
      pcl::toROSMsg(point_cloud, aux_cloud_2);
      sensor_msgs::convertPointCloud2ToPointCloud(aux_cloud_2, aux_cloud1a); 	
      listener.transformPointCloud("base_footprint",aux_cloud1a, aux_cloud1b);
      //ROS_INFO("CloudNormal size %u, cloud1 size %u", (uint32_t)cloud.points.size(),(uint32_t)aux_cloud1b.points.size() );
      planes.cloud_ = cloud;
      planes.cloud_t_=aux_cloud1b;
      //ROS_INFO("Frame origin x %.8f, y %.8f, z %.8f", obs.origin_.x, obs.origin_.y, obs.origin_.z);

      precipice_obs = -1;
      precipice_count =0;
      double precipice_threshold_aux=aux_cloud1b.points.size()*2/5; // If precipice points are not eliminated, precipice depends on point cloud size
      if ((aux_cloud1b.points.size() < precipice_threshold)){ //Precipice when points are eliminated (it depends on laser driver)
         precipice_flag=true;
      }
      for(unsigned int i = 0; i < aux_cloud1b.points.size(); ++i){

        //if precipice when points are set to their maximum value (it depends on laser driver)
        if ( (fabs(aux_cloud1b.points[i].x ) >4) || (fabs(aux_cloud1b.points[i].y) >4) || (fabs(aux_cloud1b.points[i].z) >4)) {
           precipice_count++;        
           if ((precipice_count > precipice_threshold_aux)){
              precipice_flag=true;
           }
        }
        //if precipice, generates a obstacle in front of the robot
        if(precipice_flag){
            ROS_DEBUG("Precipice");
            precipice_count = precipice_threshold+10; 
            aux_cloud1b.points[i].x=0.9;
            aux_cloud1b.points[i].y=precipice_obs;
            aux_cloud1b.points[i].z=0.5;
            precipice_obs=precipice_obs + 1; 
            if (precipice_obs >1){ 
                precipice_obs= -1;
            }    

        }  
        //if the obstacle is too high or too far away from the robot we won't add it
        if(fabs(fabs(aux_cloud1b.points[i].z)) > max_obstacle_height_)
          continue;

        //compute the squared distance from the hitpoint to the pointcloud's origin
        double sq_dist = (aux_cloud1b.points[i].x) * (aux_cloud1b.points[i].x) + (aux_cloud1b.points[i].y ) * (aux_cloud1b.points[i].y)+ (aux_cloud1b.points[i].z ) * (aux_cloud1b.points[i].z);

        //if the point is far enough away... we won't consider it
        if(sq_dist >= sq_obstacle_range){
          continue;
        }
        //compute obstacles
        if (planes.is_free(i)==true){                                                                       
          continue;
        }
        //Mark obstacle zone
        if ((aux_cloud1b.points[i].x >xmed_) && (aux_cloud1b.points[i].x <xmax_)){
           if ( (aux_cloud1b.points[i].y >ymin_) && (aux_cloud1b.points[i].y <ymed1_) ){
              zoneA_aux=true;
           }else if ((aux_cloud1b.points[i].y >ymed1_) && (aux_cloud1b.points[i].y <ymed2_)){
              zoneB_aux=true;
           }else if ((aux_cloud1b.points[i].y >ymed2_) && (aux_cloud1b.points[i].y <ymax_)){
              zoneC_aux=true;
           }
        }else if ((aux_cloud1b.points[i].x >xmin_) && (aux_cloud1b.points[i].x <xmed_)){
           if ((aux_cloud1b.points[i].y >ymin_) && (aux_cloud1b.points[i].y <ymed1_)){
              zoneD_aux=true;
           }else if ((aux_cloud1b.points[i].y >ymed1_) && (aux_cloud1b.points[i].y <ymed2_)){
              zoneE_aux=true;
           }else if ((aux_cloud1b.points[i].y >ymed2_) && (aux_cloud1b.points[i].y <ymax_)){
              zoneF_aux=true; 
           }       
        }
      }
      //Auxs are mandatory because TestVelocityCallback is faster than floorlasercallback
      if (zoneA_aux){ 
         zoneA_=true;
      }else{
         zoneA_=false;
      }
      if (zoneB_aux){ 
         zoneB_=true;
      }else{
         zoneB_=false;
      }
      if (zoneC_aux){ 
         zoneC_=true;
      }else{
         zoneC_=false;
      }
      if (zoneD_aux){ 
         zoneD_=true;
      }else{
         zoneD_=false;
      }
      if (zoneE_aux){ 
         zoneE_=true;
      }else{
         zoneE_=false;
      }
      if (zoneF_aux){ 
         zoneF_=true;
      }else{
         zoneF_=false;
      }
  }

void FrontLaserCallback (const sensor_msgs::LaserScan front_laser){

  bool zoneA2_aux = false;
  bool zoneB2_aux = false;
  bool zoneC2_aux = false;
  bool zoneD2_aux = false;
  bool zoneE2_aux = false;
  bool zoneF2_aux = false;
  sensor_msgs::PointCloud aux;
  sensor_msgs::PointCloud2 aux2;
  ros::Duration(1/frequency_).sleep();  //Due to waitForTransform does not work fine
  //listener.waitForTransform(front_laser.header.frame_id, "/base_footprint", ros::Time(), ros::Duration(2.0));
  projector_.projectLaser(front_laser, aux);
  aux.header.frame_id=front_laser.header.frame_id;
  listener.transformPointCloud("base_footprint",aux, aux);
  sensor_msgs::convertPointCloudToPointCloud2( aux, aux2); 
  //projector_.transformLaserScanToPointCloud("base_footprint", front_laser, aux, listener);
  pcl::fromROSMsg(aux2,front_laser_);


   //Now with front_laser scan
   for(unsigned int i = 0; i < front_laser_.points.size(); ++i){
        if ((front_laser_.points[i].x >xmed_) && (front_laser_.points[i].x <xmax_)){
           if ((front_laser_.points[i].y >ymin_) && (front_laser_.points[i].y <ymed1_)){
              zoneA2_aux=true;
           }else if ((front_laser_.points[i].y >ymed1_) && (front_laser_.points[i].y <ymed2_)){
              zoneB2_aux=true;
           }else if ((front_laser_.points[i].y >ymed2_) && (front_laser_.points[i].y <ymax_)){
              zoneC2_aux=true;
           }
        }else if ((front_laser_.points[i].x >xmin_) && (front_laser_.points[i].x <xmed_)){
           if (   (front_laser_.points[i].y >ymin_) && (front_laser_.points[i].y <ymed1_)){
              zoneD2_aux=true;
           }else if ((front_laser_.points[i].y >ymed1_) && (front_laser_.points[i].y <ymed2_)){
              zoneE2_aux=true;
           }else if ((front_laser_.points[i].y >ymed2_) && (front_laser_.points[i].y <ymax_)){
              zoneF2_aux=true; 

           }       
        }
    }
      if (zoneA2_aux){ 
         zoneA2_=true;
      }else{
         zoneA2_=false;
      }
      if (zoneB2_aux){ 
         zoneB2_=true;
      }else{
         zoneB2_=false;
      }
      if (zoneC2_aux){ 
         zoneC2_=true;
      }else{
         zoneC2_=false;
      }
      if (zoneD2_aux){ 
         zoneD2_=true;
      }else{
         zoneD2_=false;
      }
      if (zoneE2_aux){ 
         zoneE2_=true;
      }else{
         zoneE2_=false;
      }
      if (zoneF2_aux){ 
         zoneF2_=true;
      }else{
         zoneF2_=false;
      }


}

 void TestVelocity(const ros::TimerEvent& e){
  if(enabled_){
    current_velocity_=true;
    new_velocity_=true;
    obstacle_grid_.cells[0].z=5;
    obstacle_grid_.cells[1].z=6;
    obstacle_grid_.cells[2].z=7;
    obstacle_grid_.cells[3].z=8;
    obstacle_grid_.cells[4].z=9;
    obstacle_grid_.cells[5].z=10;
    obstacle_grid_.header.stamp=ros::Time::now();

    if (zoneA_ || zoneA2_){
      obstacle_grid_.cells[0].x=xmed_ + obstacle_grid_.cell_height/2;
      obstacle_grid_.cells[0].y=ymin_ + obstacle_grid_.cell_width/2;
      obstacle_grid_.cells[0].z=0;
	    if ( (Twist_In_.linear.x >high_vel_) && (Twist_In_.angular.z < -yaw_tolerance)){
    		current_velocity_=false;
   	  }
			if ( (WiiTwist_In_.linear.x >high_vel_) && (WiiTwist_In_.angular.z <0)){
    		new_velocity_=false;
   	  }
    }
    if (zoneB_ || zoneB2_){
      obstacle_grid_.cells[1].x=xmed_ + obstacle_grid_.cell_height/2;
      obstacle_grid_.cells[1].y=0;
      obstacle_grid_.cells[1].z=0;
      if ( (Twist_In_.linear.x >high_vel_) ){
         current_velocity_=false;
      }
      if ( (WiiTwist_In_.linear.x >high_vel_) ){
         new_velocity_=false;
      }
    }
    if (zoneC_ || zoneC2_){
      obstacle_grid_.cells[2].x=xmed_ + obstacle_grid_.cell_height/2;
      obstacle_grid_.cells[2].y=ymed2_ + obstacle_grid_.cell_width/2;
      obstacle_grid_.cells[2].z=0;
      if ( (Twist_In_.linear.x >high_vel_) && (Twist_In_.angular.z > yaw_tolerance)){
        current_velocity_=false;
      }
      if ( (WiiTwist_In_.linear.x >high_vel_) && (WiiTwist_In_.angular.z > 0)){
        new_velocity_=false;
      }
    }

    if (zoneD_ || zoneD2_){
      obstacle_grid_.cells[3].x=xmin_ + obstacle_grid_.cell_height/2;
      obstacle_grid_.cells[3].y=ymin_ + obstacle_grid_.cell_width/2;
      obstacle_grid_.cells[3].z=0;
      if ((Twist_In_.linear.x>0.001) && (Twist_In_.angular.z < -yaw_tolerance)){
        current_velocity_=false;
      }
      if ((WiiTwist_In_.linear.x>0) && (WiiTwist_In_.angular.z < 0)){
        new_velocity_=false;
      }
    }

    if (zoneE_ || zoneE2_){
      obstacle_grid_.cells[4].x=xmin_ + obstacle_grid_.cell_height/2;
      obstacle_grid_.cells[4].y=0;
      obstacle_grid_.cells[4].z=0;
      if ( (Twist_In_.linear.x >0.0000000001)){
        current_velocity_=false;
      }
      if ( (WiiTwist_In_.linear.x >0)){
        new_velocity_=false;
      }
    }

    if (zoneF_  || zoneF2_){
      obstacle_grid_.cells[5].x=xmin_ + obstacle_grid_.cell_height/2;
      obstacle_grid_.cells[5].y=ymed2_ + obstacle_grid_.cell_width/2;
      obstacle_grid_.cells[5].z=0;
      if ((Twist_In_.linear.x >0.001) && (Twist_In_.angular.z >yaw_tolerance)){
        current_velocity_=false;
      }
      if ((WiiTwist_In_.linear.x >0)&& (WiiTwist_In_.angular.z >0)){
        new_velocity_=false;
      }
    }



   if((new_velocity_) && ((WiiTwist_In_.linear.x != WiiTwist_Inold_.linear.x) || (WiiTwist_In_.angular.z != WiiTwist_Inold_.angular.z))){
      Twistoutmsgs_.linear.x=WiiTwist_In_.linear.x;
      Twistoutmsgs_.angular.z=WiiTwist_In_.angular.z;
      WiiTwist_Inold_.linear.x=WiiTwist_In_.linear.x;
      WiiTwist_Inold_.angular.z=WiiTwist_In_.angular.z;
      ROS_DEBUG("New Wii velocity");
    }else if ((new_velocity_) && (current_velocity_)){
      Twistoutmsgs_.linear.x=WiiTwist_Inold_.linear.x;
      Twistoutmsgs_.angular.z=WiiTwist_Inold_.angular.z;
    }


   if(!current_velocity_){
      ROS_WARN("Innapropiate current velocity");
      Twistoutmsgs_.linear.x=0;
      Twistoutmsgs_.angular.z=0;
    }

   pub_obstacle_grid.publish(obstacle_grid_);



  }else{
    Twistoutmsgs_=WiiTwist_In_;
  }
  ROS_DEBUG("Wii old velocity %.8f x, %.8f z, new velocity %.8f x, %.8f z", WiiTwist_Inold_.linear.x, WiiTwist_Inold_.angular.z, WiiTwist_In_.linear.x, WiiTwist_In_.angular.z);
  ROS_DEBUG("msgs old velocity %.8f x, %.8f z, new velocity %.8f x, %.8f z", Twistoutmsgsold_.linear.x, Twistoutmsgsold_.angular.z, Twistoutmsgs_.linear.x, Twistoutmsgs_.angular.z);
  ROS_DEBUG("current velocity %.8f x, %.8f z", Twist_In_.linear.x, Twist_In_.angular.z);
  if((Twistoutmsgs_.linear.x!= Twistoutmsgsold_.linear.x) || (Twistoutmsgs_.angular.z != Twistoutmsgsold_.angular.z)){
    pub_cmd_vel.publish(Twistoutmsgs_);
    Twistoutmsgsold_.linear.x = Twistoutmsgs_.linear.x;
    Twistoutmsgsold_.angular.z= Twistoutmsgs_.angular.z;
   }
  

}




void StatusCallback(const iri_segway_rmp_msgs::SegwayRMP400Status Status){

  current_time_ = ros::Time::now();
  // calculate the delta T
  double dt = (current_time_ - last_time_).toSec();

  //left turn -> positive. in rads
  double vth = - ((Status.rmp200[0].yaw_rate + Status.rmp200[1].yaw_rate) / 2);
  // First, we will only use the half of time. See hearder file for info.
  accum_th_ = vth * dt; 

  //get current translational velocity and component velocities
  double v_left_wheels  = (Status.rmp200[0].left_wheel_velocity +
                           Status.rmp200[1].left_wheel_velocity) / 2;
  double v_right_wheels = (Status.rmp200[0].right_wheel_velocity +
                           Status.rmp200[1].right_wheel_velocity) / 2;

  double vT  = (v_left_wheels + v_right_wheels) / 2;
  double vx  = vT * cos(accum_th_);

  //update second half theta
  //accum_th_ += vth * dt/2;

  Twist_In_.linear.x  = vx;
  Twist_In_.linear.y  = 0.0;
  Twist_In_.linear.z  = 0.0;
  Twist_In_.angular.x = 0.0;
  Twist_In_.angular.y = 0.0;
  Twist_In_.angular.z = vth;

  //update last time
  last_time_ = current_time_;
}

void WiiMoteCallback (geometry_msgs::Twist WiiTwist){
   WiiTwist_In_=WiiTwist;
}




  const tf::TransformListener& listener;

private:
  ros::NodeHandle n_;
  ros::Publisher pub_cmd_vel;
  ros::Publisher pub_obstacle_grid;
  laser_geometry::LaserProjection projector_;

  int precipice_threshold;
  geometry_msgs::Twist Twist_In_; 
  geometry_msgs::Twist WiiTwist_In_;
  geometry_msgs::Twist WiiTwist_Inold_;
  geometry_msgs::Twist Twistoutmsgsold_;
  geometry_msgs::Twist Twistoutmsgs_;
  pcl::PointCloud<pcl::PointXYZ> front_laser_;
  iri_segway_rmp_msgs::SegwayRMP400Status Status_;
  ros::Time current_time_;
  ros::Time last_time_;
  double accum_th_;
  double max_obstacle_height_;
  bool first_time_;
  double obstacle_range_;
  bool zoneA_;
  bool zoneB_;
  bool zoneC_;
  bool zoneD_;
  bool zoneE_;
  bool zoneF_;
  bool zoneA2_;
  bool zoneB2_;
  bool zoneC2_;
  bool zoneD2_;
  bool zoneE2_;
  bool zoneF2_;
  nav_msgs::GridCells obstacle_grid_;
  bool enabled_;
  bool downsampling_;
  bool current_velocity_;
  bool new_velocity_;
  costmap_3d planes;
  double xmax_;
  double xmed_;
  double xmin_;
  double ymax_;
  double ymed1_;
  double ymed2_;
  double ymin_;
  ros::Timer timer_;
  double frequency_;
  double yaw_tolerance;

  //Velocity constraints
  double soft_turn_;
  double tough_turn_;
  double low_vel_;
  double high_vel_;



} ;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_controller");
  ros::NodeHandle n;
  tf::TransformListener listener(ros::Duration(10));
  Nav_Controller nav_controller(listener);

  ros::Subscriber sub =n.subscribe<pcl::PointCloud<pcl::PointNormal> >("assembled_cloud_with_normals", 10, &Nav_Controller::FloorLaserCallback, &nav_controller);
  ros::Subscriber sub_front =n.subscribe<sensor_msgs::LaserScan>("teo/front_laser/scan", 10, &Nav_Controller::FrontLaserCallback, &nav_controller);
  ros::Subscriber sub_vel =n.subscribe<geometry_msgs::Twist >("/WiiTwist", 10, &Nav_Controller::WiiMoteCallback, &nav_controller);
  ros::Subscriber sub_r =n.subscribe<iri_segway_rmp_msgs::SegwayRMP400Status>("teo/segway/status", 10, &Nav_Controller::StatusCallback, &nav_controller);
  ros::spin();
  return 0;
}
