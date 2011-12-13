#include <ros/ros.h>

#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/Shape.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "vertical_stick");

  ros::NodeHandle nh;

  ros::Publisher object_in_map_pub_;
  object_in_map_pub_  = nh.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);

  sleep(2);

  //add the cylinder into the collision space
  arm_navigation_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "/world";
  cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "/wambase";
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
  object.dimensions[0]=0.07;
  object.dimensions[1]=13.2;
  geometry_msgs::Pose pose;
  pose.position.x=0.5;
  pose.position.y=0.14;
  pose.position.z=0.01;
  pose.orientation.x=0;
  pose.orientation.y=0;
  pose.orientation.z=0;
  pose.orientation.w=1;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);

  object_in_map_pub_.publish(cylinder_object);

  ROS_INFO("Should have published");

  ros::Duration(2.0).sleep();

  ros::shutdown();

}
