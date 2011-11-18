#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <std_srvs/Empty.h>
int main(int argc, char **argv){
	static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/register_planning_scene";

  ros::init (argc, argv, "test_move_iri_wam");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_iri_wam",true);
  move_arm.waitForServer();
  ROS_INFO("Connected to server");
  arm_navigation_msgs::MoveArmGoal goalA;

  goalA.planner_service_name="/ompl_planning/plan_kinematic_path";
  goalA.motion_plan_request.group_name="iri_wam";
  arm_navigation_msgs::JointConstraint joint1;
  joint1.joint_name="j1";
  joint1.position=1.0;
  joint1.tolerance_above=0.01;
  joint1.tolerance_below=0.01;
  joint1.weight=1.0;
  arm_navigation_msgs::JointConstraint joint2;
  joint2.joint_name="j2";
  joint2.position=0.0;
  joint2.tolerance_above=0.01;
  joint2.tolerance_below=0.01;
  joint2.weight=1.0;  
  arm_navigation_msgs::JointConstraint joint3;
  joint3.joint_name="j3";
  joint3.position=0.0;
  joint3.tolerance_above=0.01;
  joint3.tolerance_below=0.01;
  joint3.weight=1.0;  
  arm_navigation_msgs::JointConstraint joint4;
  joint4.joint_name="j4";
  joint4.position=0.5;
  joint4.tolerance_above=0.01;
  joint4.tolerance_below=0.01;
  joint4.weight=1.0;  
  arm_navigation_msgs::JointConstraint joint5;
  joint5.joint_name="j5";
  joint5.position=0.0;
  joint5.tolerance_above=0.01;
  joint5.tolerance_below=0.01;
  joint5.weight=1.0;  
  arm_navigation_msgs::JointConstraint joint6;
  joint6.joint_name="j6";
  joint6.position=0.0;
  joint6.tolerance_above=0.01;
  joint6.tolerance_below=0.01;
  joint6.weight=1.0;  
  arm_navigation_msgs::JointConstraint joint7;
  joint7.joint_name="j7";
  joint7.position=0.0;
  joint7.tolerance_above=0.01;
  joint7.tolerance_below=0.01;
  joint7.weight=1.0;  

  goalA.motion_plan_request.group_name = "iri_wam";
  goalA.motion_plan_request.num_planning_attempts = 1;
  //goalA.motion_plan_request.planner_id = std::string("");
  //goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  goalA.motion_plan_request.goal_constraints.joint_constraints.resize(7);
  goalA.motion_plan_request.goal_constraints.joint_constraints[0]=joint1;
  goalA.motion_plan_request.goal_constraints.joint_constraints[1]=joint2;
  goalA.motion_plan_request.goal_constraints.joint_constraints[2]=joint3;
  goalA.motion_plan_request.goal_constraints.joint_constraints[3]=joint4;
  goalA.motion_plan_request.goal_constraints.joint_constraints[4]=joint5;
  goalA.motion_plan_request.goal_constraints.joint_constraints[5]=joint6;
  goalA.motion_plan_request.goal_constraints.joint_constraints[6]=joint7;


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
