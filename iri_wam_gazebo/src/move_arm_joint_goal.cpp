#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <sstream>
#include <iostream>
#include <stdlib.h>

int main(int argc, char **argv){
  
  ros::init (argc, argv, "move_arm_joint_goal");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_iri_wam",true);

  move_arm.waitForServer();
  ROS_INFO("Connected to server");

  arm_navigation_msgs::MoveArmGoal goalA;
  std::vector<std::string> names(7);
  names[0] = "j1_joint";
  names[1] = "j2_joint";
  names[2] = "j3_joint";
  names[3] = "j4_joint";
  names[4] = "j5_joint";
  names[5] = "j6_joint";
  names[6] = "j7_joint";

  goalA.motion_plan_request.group_name="iri_wam";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name=std::string("ompl_planning/plan_kinematic_path");
  goalA.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());

  for (unsigned int i = 0 ; i < goalA.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    goalA.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
    goalA.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.01;
    goalA.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.01;
    goalA.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.01;
    // goalA.motion_plan_request.goal_constraints.joint_constraints[i].weight = 1.0;
  }

  goalA.motion_plan_request.goal_constraints.joint_constraints[0].position = strtod(argv[1],NULL);
  goalA.motion_plan_request.goal_constraints.joint_constraints[1].position = strtod(argv[2],NULL);
  goalA.motion_plan_request.goal_constraints.joint_constraints[2].position = strtod(argv[3],NULL);
  goalA.motion_plan_request.goal_constraints.joint_constraints[3].position = strtod(argv[4],NULL);
  goalA.motion_plan_request.goal_constraints.joint_constraints[4].position = strtod(argv[5],NULL);
  goalA.motion_plan_request.goal_constraints.joint_constraints[5].position = strtod(argv[6],NULL);
  goalA.motion_plan_request.goal_constraints.joint_constraints[6].position = strtod(argv[7],NULL);

  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  } 
  ros::shutdown();
}
