#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
  // start a ROS spinning thread this allows finish the trajectory planner
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // verify that there are set the seven joints
  if (argc != 8)
  {
    ROS_INFO("usage: move_group_interface_demo joint1 joint2 joint3 joint4 joint5 joint6 joint7");
    return 1;
  }
  // define which members of the descripition would be moved
  move_group_interface::MoveGroup group("arm");

  //if we want to set a different planner (by default Moveit! uses KPIECE) --> group.setPlannerId("RRTConnectkConfigDefault");

  // define the value of the joints of the Goal Position
  std::vector<double> pose(7, 0.0);
  for (int i = 0; i < (int) pose.size(); ++i) 
    pose[i] = atof(argv[i+1]);
  group.setJointValueTarget(pose);  

  // choose the favourite trajectory
  std::string response = "no";
  move_group_interface::MoveGroup::Plan trajectory;
  while (response == "no") 
  {
    group.plan(trajectory);
    std::cout << "Do you like the plan? (yes/no/quit): ";
    std::cin >> response;
    while (response != "yes" and response != "no" and response != "quit")
    {
      std::cout << "Say (yes/no/quit): ";
      std::cin >> response;
    }
  }
  // execute the chosen trajectory
  if (response == "yes") group.execute(trajectory);
  
  return 0;
}
