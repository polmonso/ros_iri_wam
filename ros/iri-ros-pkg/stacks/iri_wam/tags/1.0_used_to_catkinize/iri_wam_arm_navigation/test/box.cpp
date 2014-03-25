#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/Shape.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "box");

  ros::NodeHandle nh;

  ros::Publisher object_in_map_pub_;
  object_in_map_pub_  = nh.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);

  sleep(2);

  //add the cylinder into the collision space
  arm_navigation_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "/boxs";
  // COllision Operation
  // ADD
  //REMOVE
  // DETACH_AND_ADD_AS_OBJECT
  //ATTACH_AND_REMOVE_AS_OBJECT
  cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "/world";
  cylinder_object.header.stamp = ros::Time::now();
  arm_navigation_msgs::Shape object;
  //shape type
  //CYLINDER
  //SPHERE
  //BOX
  //MESH
  object.type = arm_navigation_msgs::Shape::BOX;

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

 /* object.dimensions.resize(2);
  object.dimensions[0] = 0.02;
  object.dimensions[1] = 1.5;
  geometry_msgs::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = 0.14;
  pose.position.z = 0.75;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;*/
  object.dimensions.resize(3);
  object.dimensions[0] = 0.25;
  object.dimensions[1] = 0.25;
  object.dimensions[2] = 0.5;
  geometry_msgs::Pose pose;
  pose.position.x = 0.53;
  pose.position.y = 0.14;
  pose.position.z = 0.75;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);

  object_in_map_pub_.publish(cylinder_object);

  ROS_INFO("Should have published");

  ros::Duration(2.0).sleep();

  ros::shutdown();

}
