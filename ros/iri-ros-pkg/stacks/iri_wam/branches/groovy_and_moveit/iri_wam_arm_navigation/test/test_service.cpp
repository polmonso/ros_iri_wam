#include <ros/ros.h>
#include "iri_wam_arm_navigation/PosePath.h"
int main(int argc, char ** argv)
{
	ros::init(argc,argv,"test");
  iri_wam_arm_navigation::PosePath::Request request;
  iri_wam_arm_navigation::PosePath::Response response;
  double tx,ty,tz,qx,qy,qz,qw;
  ROS_INFO("Ingresa X de traslacion");	
  std::cin>>tx;
  request.pose.position.x=tx;
  ROS_INFO_STREAM("X ingresada: "<<tx);	
  ROS_INFO("Ingresa Y de traslacion");	
  std::cin>>ty;
  request.pose.position.y=ty;
  ROS_INFO_STREAM("Y ingresada: "<<ty);	
  ROS_INFO("Ingresa Z de traslacion");	
  std::cin>>tz;
  request.pose.position.z=tz;
  ROS_INFO_STREAM("Z ingresada: "<<tz);	
  ROS_INFO("Ingresa X de Rotacion");	
  std::cin>>qx;
  request.pose.orientation.x=qx;
  ROS_INFO_STREAM("X ingresada: "<<qx);	
  ROS_INFO("Ingresa Y de Rotacion");	
  std::cin>>qy;
  request.pose.orientation.y=qy;
  ROS_INFO_STREAM("Y ingresada: "<<qy);	
  ROS_INFO("Ingresa Z de Rotacion");	
  std::cin>>qz;
  request.pose.orientation.z=qz;
  ROS_INFO_STREAM("Z ingresada: "<<qz);	
  ROS_INFO("Ingresa W de Rotacion");	
  std::cin>>qw;
  request.pose.orientation.w=qw;
  ROS_INFO_STREAM("W ingresada: "<<qw);	
 
  ros::NodeHandle nh;
  ros::service::waitForService("/get_pose_path");	
  ros::ServiceClient client = nh.serviceClient<iri_wam_arm_navigation::PosePath>("/get_pose_path");
	
   if(client.call(request, response))
   { 
     if(response.solution_found == true)
     {
       
       ROS_INFO("[iri_wam_arm_navigation]Found Solution");
       for(int i=0; i < response.poses_solution.size(); ++i)
       {
         ROS_INFO_STREAM(""<<response.poses_solution[i]);
        }
     }
     else
     {
       ROS_ERROR("[iri_wam_arm_navigation]No Found Solution");
     } 
   }
   else
   {
     ROS_ERROR("[iri_wam_arm_navigation]service call failed");
      ros::shutdown();
   }
	
	return 0;
}
