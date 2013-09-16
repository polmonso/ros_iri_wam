#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& jointState)
{
  // print what is published by /joint_states
  std::vector<double> pose = jointState->position;
  std::cout << "The joint state is: ";
  for(int i=0; i < (int) pose.size(); ++i)
    std::cout << (i==0 ? "" : " " ) << pose.at(i);
  std::cout << std::endl;
  // shut down the node
  ros::shutdown();
}

int main(int argc, char **argv)
{
  // start the node
  ros::init(argc, argv, "joint_state");
  ros::NodeHandle n;
  // subscribe this node to the publisher /joint_states
  ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("joint_states", 1, jointStateCallback);
  ros::spin();

  return 0;
}
