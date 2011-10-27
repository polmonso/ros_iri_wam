#include "obj_filter.h"

using namespace std;

ObjectFilter::ObjectFilter() {

  //init class attributes if necessary
  this->transform_grasping_point.setOrigin(tf::Vector3(0.5, 0.0, 0.5));
  this->transform_grasping_point.setRotation(tf::Quaternion(0, 0, 0, sqrt(2)/2));
  this->focused_obj = 0;

  //string for port names
  std::string pcl2_sub_name               = "/obj_filter/input/pcl2/segmented"; //segmented pcl input //should be raw when in conjunction with image/segmented 
  std::string heat_map_sub_name           = "/obj_filter/input/pcl2/wrinkled_map"; //result from normal_descriptors_node
  std::string segmented_img_sub_name      = "/obj_filter/input/image/segmented"; //not used
  std::string focused_obj_sub_name        = "/obj_filter/input/focused_obj"; //selected object label
  std::string filtered_pcl_sub_name       = "/obj_filter/output/pcl2/filtered"; //pcl2 input of normal_descriptors_node
  std::string compute_grasp_pose_sub_name = "/obj_filter/srv/compute_grasp_pose"; //service that triggers the normal_descriptors_node call

  // [init subscribers]
  this->pcl2_sub = this->nh.subscribe(pcl2_sub_name, 1, &ObjectFilter::pcl2_sub_callback, this);
  this->segmented_img_sub = this->nh.subscribe(segmented_img_sub_name, 1, &ObjectFilter::segmented_img_sub_callback, this);
  this->heat_map_sub = this->nh.subscribe(heat_map_sub_name, 1, &ObjectFilter::heat_map_sub_callback, this);
  this->focused_object_sub = this->nh.subscribe(focused_obj_sub_name, 1, &ObjectFilter::focused_obj_sub_callback, this);
  
  // [init publishers]
  this->filtered_pcl2_publisher = this->nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> > (filtered_pcl_sub_name, 5);

  // [init services]
  this->compute_grasp = this->nh.advertiseService(compute_grasp_pose_sub_name, &ObjectFilter::compute_grasp_pose_callback, this);
  
  // [init clients]
  this->wrinkle_client = this->nh.serviceClient<normal_descriptor_node::wrinkle>("/obj_filter/output/srv/wrinkle_map");
  
  // [init action servers]
  
  // [init action clients]
  ROS_INFO("Starting obj_filter Node");
  ROS_INFO("Use [ rosservice call /compute_grasp_pose ] to trigger the normal descriptor");
}

/*  [subscriber callbacks] */
void ObjectFilter::pcl2_sub_callback(const sensor_msgs::PointCloud2ConstPtr& msg) 
{ 
  ROS_DEBUG("Received segmented point cloud");
  pcl::fromROSMsg(*msg, this->pcl_xyzrgb);

//  int index0 = 0;
//  xi = new float[ msg->width * msg->height ];
//  yi = new float[ msg->width * msg->height ];
//  zi = new float[ msg->width * msg->height ];
//  ci = new float[ msg->width * msg->height ];
//  for(int ii=0; ii<msg->height; ii++){
//    for(int jj=0; jj<msg->width; jj++){
//      int kk = ii*msg->width + jj;
//      float *pstep2 = (float*)&msg->data[(kk) * msg->point_step];
//      xi[kk] = pstep2[0]; //x
//      yi[kk] = pstep2[1]; //y
//      zi[kk] = pstep2[2]; //z
//      ci[kk] = pstep2[3]; //rgb 
//      index0++;
//    }
//  } 

}

void ObjectFilter::mainLoop(void)
{
    tf_br.sendTransform(tf::StampedTransform(transform_grasping_point, ros::Time::now(), "openni_depth_optical_frame", "graspingPoint"));
}

void ObjectFilter::heat_map_sub_callback(const normal_descriptor_node::heat_map& msg) 
{ 
  ROS_INFO("Received heat map");
  int indx = 0;
  int max = 0;
  for(int i=0; i<msg.data.size();i++){
    if(max < msg.data[i]){
      max = msg.data[i];
      indx = i; 
    }
  }
  graspPose.header.stamp = ros::Time::now();
  graspPose.header.frame_id = "grasp_point";
  graspPose.pose.position.x = msg.points3d[indx].x;
  graspPose.pose.position.y = msg.points3d[indx].y;
  graspPose.pose.position.z = msg.points3d[indx].z;

  graspPose.pose.orientation.x = 0;
  graspPose.pose.orientation.y = 0;
  graspPose.pose.orientation.z = 0;
  graspPose.pose.orientation.w = sqrt(2)/2;

  ROS_INFO("Grasping point: %f %f %f ",graspPose.pose.position.x, graspPose.pose.position.y, graspPose.pose.position.z);

}

void ObjectFilter::focused_obj_sub_callback(const std_msgs::Int32& msg) 
{ 

}

void ObjectFilter::segmented_img_sub_callback(const sensor_msgs::ImageConstPtr& msg) 
{ 

  //this callback will be unused while we transport the RGB segmented image with the pointcloud

//  int index0 = 0;
//  *ai = new float[ msg->width * msg->height ];
//  int kk = 0;
//  for (int ii=0; ii<msg->height; ii++) {
//    for (int jj=0; jj<msg->width; jj++) {
//      kk = ii*msg->width + jj;
//      ai[kk] = (float)msg->data[kk];
//      index0++;
//    }
//  }

}

/*  [service callback] */
bool ObjectFilter::compute_grasp_pose_callback(iri_wam_common_msgs::compute_obj_grasp_pose::Request &req, iri_wam_common_msgs::compute_obj_grasp_pose::Response &res){

    ROS_INFO("Compute grasp pose request");

    //assuming shared object pcl_xyzrgb integrity !
    pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = this->pcl_xyzrgb.begin();
    RGBValue color;
    switch(req.filterID){
        case ANYZONE:
            //don't filter
            break;
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
            ROS_INFO("Color filtering");
            for (int rr = 0; rr < (int)pcl_xyzrgb.height; ++rr) {
              for (int cc = 0; cc < (int)pcl_xyzrgb.width; ++cc, ++pt_iter) {
        	    color.float_value = pt_iter->rgb;
                if((int)color.Red != req.filterID) {
                  pt_iter->x = NAN; 
                  pt_iter->y = NAN; 
                  pt_iter->z = NAN; 
                }
              }
            }
            break;
        case AZONE: //only zone A
            ROS_INFO("only zone A");
            for (int rr = 0; rr < (int)pcl_xyzrgb.height; ++rr) {
              pt_iter += pcl_xyzrgb.width/2;
              for (int cc = 0; cc < (int)(pcl_xyzrgb.width/2); ++cc, ++pt_iter) {
                  pt_iter->x = NAN; 
                  pt_iter->y = NAN; 
                  pt_iter->z = NAN; 
              }
            }
            break;
        case BZONE: //only zone B
            ROS_INFO("only zone B");
            for (int rr = 0; rr < (int)pcl_xyzrgb.height; ++rr) {
              for (int cc = 0; cc < (int)(pcl_xyzrgb.width/2); ++cc, ++pt_iter) {
                  pt_iter->x = NAN; 
                  pt_iter->y = NAN; 
                  pt_iter->z = NAN; 
              }
              pt_iter += pcl_xyzrgb.width/2;
            }
            break;
        default:
            ROS_ERROR("Unrecognized filter criteria");
            return false;
            break;
    }

    //assuming pcl_xyzrgb integrity !
    this->filtered_pcl2_publisher.publish(this->pcl_xyzrgb);
    ROS_INFO("Calling wrinkle service");
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(this->pcl_xyzrgb, cloud);
    wrinkle_srv.request.cloth_cloud = cloud; 
    if(wrinkle_client.call(wrinkle_srv)) { 
        ROS_INFO("call to wrinkle srv returned successfully"); 
    } else { 
      ROS_ERROR("Failed to call service wrinkle_srv. Reason [todo]"); 
      return false;
    }
    ROS_INFO("Received heat map");
    int indx = 0;
    double max = 0;
    int counter = 0;
    for(int i=0; i<wrinkle_srv.response.wrinkle_map.data.size();i++){
    //TODO add within workspace condition
      if(!isnan(wrinkle_srv.response.wrinkle_map.points3d[i].x) && !isnan(wrinkle_srv.response.wrinkle_map.points3d[i].y) && !isnan(wrinkle_srv.response.wrinkle_map.points3d[i].z) ){
          counter++;
          if(max < wrinkle_srv.response.wrinkle_map.data[i]){
            max = wrinkle_srv.response.wrinkle_map.data[i];
            indx = i; 
         }
      }
    }
    if(max == 0){
        ROS_ERROR("no max wrinkle found! Candidates were all nan or no maximum diferent than 0 was found");
        return false;
    }else{
        ROS_INFO("valid wrinkle point found at %f %f %f with wrinkleness %f",wrinkle_srv.response.wrinkle_map.points3d[indx].x,wrinkle_srv.response.wrinkle_map.points3d[indx].y,wrinkle_srv.response.wrinkle_map.points3d[indx].z, max); 
    }
    graspPose.header.stamp = ros::Time::now();
    graspPose.header.frame_id = "grasp_point";
    graspPose.pose.position.x = wrinkle_srv.response.wrinkle_map.points3d[indx].x;
    graspPose.pose.position.y = wrinkle_srv.response.wrinkle_map.points3d[indx].y;
    graspPose.pose.position.z = wrinkle_srv.response.wrinkle_map.points3d[indx].z;

    graspPose.pose.orientation.x = 0;
    graspPose.pose.orientation.y = 0;
    graspPose.pose.orientation.z = 0;
    graspPose.pose.orientation.w = sqrt(2)/2;

    //refresh published tf data
    transform_grasping_point.setOrigin(tf::Vector3(graspPose.pose.position.x, graspPose.pose.position.y, graspPose.pose.position.z));
    transform_grasping_point.setRotation(tf::Quaternion( graspPose.pose.position.x, graspPose.pose.orientation.y,graspPose.pose.orientation.w));

    ROS_INFO("Grasping point: %f %f %f wrinkleness %f",graspPose.pose.position.x, graspPose.pose.position.y, graspPose.pose.position.z, max);

    //grabpose
    res.graspPose = graspPose;
    res.wrinkleness = max;
    //additionally, we can publish the grasping point as tf
    //TODO publish tf
    return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "obj_filter");
  ObjectFilter handeye_log;
  ros::Rate loop_rate(10); 
  while(ros::ok()){
    handeye_log.mainLoop();
    ros::spinOnce();
    loop_rate.sleep(); 
  }
  return 0;
}
