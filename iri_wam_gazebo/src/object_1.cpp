#include <ros/ros.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include "iri_wam_common_msgs/GeneralState.h"
#include <geometry_msgs/Pose.h>


static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

int main(int argc, char **argv){
  ros::init (argc, argv, "object_add");
  ros::NodeHandle rh;
  
  geometry_msgs::Pose pose;
  pose.position.x=0.5;
  pose.position.y=0.14;
  pose.position.z=0.01;
  pose.orientation.x=0;
  pose.orientation.y=0;
  pose.orientation.z=0;
  pose.orientation.w=1;
  
  arm_navigation_msgs::Shape objeto;
  objeto.type=arm_navigation_msgs::Shape::CYLINDER;
  //objeto.dimensions.resize(2);
  objeto.dimensions.push_back(0.02);
  objeto.dimensions.push_back(1.2);
    
  arm_navigation_msgs::CollisionObject Addobjeto;
  
  Addobjeto.header.stamp=ros::Time::now();
  Addobjeto.header.frame_id="wambase";
  
  Addobjeto.id="Vara";
  Addobjeto.padding=0.01;
  Addobjeto.operation.operation=0;
  
//  Addobjeto.shapes.resize(1);
  Addobjeto.shapes.push_back(objeto);

  //Addobjeto.poses.resize(1);
  Addobjeto.poses.push_back(pose);
  
  arm_navigation_msgs::PlanningScene scene;
  //scene.collision_objects.resize(1);
  scene.collision_objects.push_back(Addobjeto);
  
   
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
  ros::ServiceClient set_planning_scene_client = rh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);

  iri_wam_common_msgs::GeneralState::Request  req;
  iri_wam_common_msgs::GeneralState::Response res;
  
  req.index=1;
  
  ros::service::waitForService("/gazebo/GeneralState");	
  ros::ServiceClient client = rh.serviceClient<iri_wam_common_msgs::GeneralState>("/gazebo/GeneralState");
  if(client.call(req,res))
  {
	  ROS_WARN("Ok");
	  scene.robot_state=res.robot_state;
	  
  }
  else ROS_ERROR("Don't have General State");
  
  
    arm_navigation_msgs::SetPlanningSceneDiff::Request request;
  arm_navigation_msgs::SetPlanningSceneDiff::Response response;
  
  request.planning_scene_diff=scene;
  while(true){
    if(!set_planning_scene_client.call(request,response)) {
    ROS_WARN("Can't get planning scene");
    return -1;
  }
  else{ROS_INFO("Thas OK");}

 }


  ros::shutdown();
}


