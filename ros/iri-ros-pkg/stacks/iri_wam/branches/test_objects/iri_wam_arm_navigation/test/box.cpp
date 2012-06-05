#include <ros/ros.h>

#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/Shape.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "vertical_stick");

  ros::NodeHandle nh("~");
  ros::NodeHandle pub("/");

  XmlRpc::XmlRpcValue val1;
  double tx,ty,tz,qx,qy,qz,qw;
  
  nh.getParam("posicion_x",val1); 
  tx=(double)val1;
  ROS_WARN_STREAM("posicion_x: "<<tx); 
  
  nh.getParam("posicion_y",val1); 
  ty=(double)val1;
  ROS_WARN_STREAM("posicion_y: "<<ty); 
  
  nh.getParam("posicion_z",val1); 
  tz=(double)val1;
  ROS_WARN_STREAM("posicion_z: "<<tz); 
  
  nh.getParam("rotacion_x",val1); 
  qx=(double)val1;
  ROS_WARN_STREAM("rotacion_x: "<<qx); 
  
  nh.getParam("rotacion_y",val1); 
  qy=(double)val1;
  ROS_WARN_STREAM("rotacion_y: "<<qy); 
  
  nh.getParam("rotacion_z",val1); 
  qz=(double)val1;
  ROS_WARN_STREAM("rotacion_z: "<<qz); 
  
  nh.getParam("rotacion_w",val1); 
  qw=(double)val1;
  ROS_WARN_STREAM("rotacion_w: "<<qw); 

  double ancho,alto,largo;
  nh.getParam("width",val1); 
  ancho=(double)val1;
  ROS_WARN_STREAM("width: "<<ancho); 
  nh.getParam("length",val1); 
  largo=(double)val1;
  ROS_WARN_STREAM("length: "<<largo); 
  nh.getParam("height",val1); 
  alto=(double)val1;
  ROS_WARN_STREAM("height: "<<alto); 
  std::string name_frame;
  nh.getParam("name_obstacle",val1); 
  name_frame=(std::string)val1;
  ROS_WARN_STREAM("name_obstacle: "<<name_frame);   
  
  ros::Publisher object_in_map_pub_;
  object_in_map_pub_  = pub.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);



  //add the cylinder into the collision space
  arm_navigation_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "/"+name_frame;
  // COllision Operation
  // ADD 
  //REMOVE
  // DETACH_AND_ADD_AS_OBJECT
  //ATTACH_AND_REMOVE_AS_OBJECT
  cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "/world";
  cylinder_object.header.stamp = ros::Time::now();
  arm_navigation_msgs::Shape object;
  object.type = arm_navigation_msgs::Shape::BOX;

// the origin of each shape is considered at the shape's center

// for sphere
// radius := dimensions[0]

//for cylinder
// radius := dimensions[0]
// length := dimensions[1]
// the length is along the Z axis

// for box
// size_x := dimensions[0]//largo
// size_y := dimensions[1]//ancho
// size_z := dimensions[2]//alto

  object.dimensions.resize(3);
  object.dimensions[0] = largo;
  object.dimensions[1] = ancho;
  object.dimensions[2] = alto;
  
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
 
 /* nh.deleteParam("posicion_x"); 
  nh.deleteParam("posicion_y"); 
  nh.deleteParam("posicion_z"); 
  nh.deleteParam("rotacion_x"); 
  nh.deleteParam("rotacion_y"); 
  nh.deleteParam("rotacion_z"); 
  nh.deleteParam("rotacion_w"); 
  nh.deleteParam("width"); 
  nh.deleteParam("length"); 
  nh.deleteParam("height"); 
  nh.deleteParam("name_obstacle");  
  */
   
        ros::Duration(5.0).sleep();
      object_in_map_pub_.publish(cylinder_object);
  ROS_INFO("Should have published");
  
    ros::Duration(5.0).sleep();
  ros::shutdown();

}
