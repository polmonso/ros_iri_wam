#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/Shape.h>

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "vertical_stick");

  ros::NodeHandle nh("~");

  ros::Publisher object_in_map_pub_;
  object_in_map_pub_  = nh.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);
  
  XmlRpc::XmlRpcValue val1;
  double rad,length;
  bool rpy,quatern;
  /*nh.getParam("rpy",val1); 
  rpy=(bool)val1;
  nh.getParam("quaternion",val1); 
  quatern=(bool)val1;
  ROS_ERROR_STREAM("RPY "<<rpy);*/
  
  nh.param<bool>("rpy", rpy, false);
  nh.param<bool>("quaternion", quatern, false);
  nh.param<double>("radius", rad, 0.01);
  nh.param<double>("length", rad, 0.01);
  
  double tx,ty,tz,qx,qy,qz,qw;
  
  nh.param<double>("posicion_x", tx, 0.01);
  ROS_WARN_STREAM("posicion_x: "<<tx); 
  
  nh.param<double>("posicion_y", ty, 0.01);
  ROS_WARN_STREAM("posicion_y: "<<ty); 
  
  nh.param<double>("posicion_z", tz, 0.01);
  ROS_WARN_STREAM("posicion_z: "<<tz); 
  
  if(quatern){
	  nh.param<double>("rotacion_x", qx, 0.01);
	  ROS_WARN_STREAM("rotacion_x: "<<qx); 
	  
	  nh.param<double>("rotacion_y", qy, 0.01);
	  ROS_WARN_STREAM("rotacion_y: "<<qy); 
	  
	  nh.param<double>("rotacion_z", qz, 0.01);
	  ROS_WARN_STREAM("rotacion_z: "<<qz); 
	  
	  nh.param<double>("rotacion_w", qw, 0.01);
	  ROS_WARN_STREAM("rotacion_w: "<<qw); 
  }
  else if(rpy)
  {
	double r,p,y;
	  nh.param<double>("roll", r, 0.01);
	  ROS_WARN_STREAM("Roll: "<<r); 
	  
	  nh.param<double>("pitch", p, 0.01);
	  ROS_WARN_STREAM("Pitch: "<<p); 
	  
	  nh.param<double>("yaw", y, 0.01);
	  ROS_WARN_STREAM("Yaw: "<<y); 
	  
	  KDL::Rotation handRotation= KDL::Rotation::RPY(r,p,y);
      handRotation.GetQuaternion(qx,qy,qz,qw);
      ROS_WARN_STREAM("rotacion_x: "<<qx); 
      ROS_WARN_STREAM("rotacion_y: "<<qy); 
      ROS_WARN_STREAM("rotacion_z: "<<qz); 
      ROS_WARN_STREAM("rotacion_w: "<<qw); 
  }
  
  std::string name_frame;
  nh.param<std::string>("name_obstacle", name_frame, "loool");
  ROS_WARN_STREAM("name_obstacle: "<<name_frame);   

  //add the cylinder into the collision space
  arm_navigation_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "/"+name_frame;
  cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "/world";
  cylinder_object.header.stamp = ros::Time::now();
  arm_navigation_msgs::Shape object;
  object.type = arm_navigation_msgs::Shape::CYLINDER;
  //////// define sphere, box, cylinder //###
// the origin of each shape is considered at the shape's center

// for sphere
// radius := dimensions[0]

//for cylinder
// radius := dimensions[0]
// length := dimensions[1]
// the length is along the Z axis

// for box
// size_x := dimensions[0]
// size_y := dimensions[1]
// size_z := dimensions[2]

  object.dimensions.resize(2);
  object.dimensions[0] = rad;
  object.dimensions[1] = length;
  geometry_msgs::Pose pose;
  pose.position.x = tx;
  pose.position.y = ty;
  pose.position.z = tz;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);

  object_in_map_pub_.publish(cylinder_object);

  ROS_INFO("Should have published");
  ros::Duration(5.0).sleep();
  ros::shutdown();
}
