#include <ros/ros.h>
#include <arm_navigation_msgs/GetMotionPlan.h>

#include <planning_environment/monitors/joint_state_monitor.h>
#include <boost/thread.hpp>

void spinThread()
{
  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "ompl_pose_goal");
  boost::thread spin_thread(&spinThread);

  ros::NodeHandle nh;

 arm_navigation_msgs::GetMotionPlan::Request request;
  arm_navigation_msgs::GetMotionPlan::Response response;

  request.motion_plan_request.group_name = "right_arm";
  request.motion_plan_request.num_planning_attempts = 1;
  request.motion_plan_request.planner_id = std::string("");
  request.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  
  request.motion_plan_request.goal_constraints.position_constraints.resize(1);
  request.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  request.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "torso_lift_link";
    
  request.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
  request.motion_plan_request.goal_constraints.position_constraints[0].position.x = 0.75;
  request.motion_plan_request.goal_constraints.position_constraints[0].position.y = -0.188;
  request.motion_plan_request.goal_constraints.position_constraints[0].position.z = 0;
    
  request.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = geometric_shapes_msgs::Shape::BOX;
  request.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  request.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  request.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);

  request.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  request.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;

  request.motion_plan_request.goal_constraints.orientation_constraints.resize(1);
  request.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  request.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "torso_lift_link";    
  request.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
  request.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = 0.0;
  request.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = 0.0;
  request.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = 0.0;
  request.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = 1.0;
    
  request.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.04;
  request.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.04;
  request.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.04;

  request.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0;


  ros::ServiceClient service_client = nh.serviceClient<motion_planning_msgs::GetMotionPlan>("ompl_planning/plan_kinematic_path");
  service_client.call(request,response);
  if(response.error_code.val != response.error_code.SUCCESS)
  {
    ROS_ERROR("Motion planning failed");
  }
  else
  {
    ROS_INFO("Motion planning succeeded");
  }


  planning_environment::JointStateMonitor joint_state_monitor;
  ros::Publisher display_trajectory_publisher = nh.advertise<motion_planning_msgs::DisplayTrajectory>("joint_path_display", 1);
  while(display_trajectory_publisher.getNumSubscribers() < 1 && nh.ok())
  {
    ROS_INFO("Waiting for subscriber");
    ros::Duration(0.1).sleep();
  }
  motion_planning_msgs::DisplayTrajectory display_trajectory;

  display_trajectory.model_id = "pr2";
  display_trajectory.trajectory.joint_trajectory.header.frame_id = "base_footprint";
  display_trajectory.trajectory.joint_trajectory.header.stamp = ros::Time::now();
  display_trajectory.robot_state.joint_state =  joint_state_monitor.getJointStateRealJoints();
  display_trajectory.trajectory = response.trajectory;
  ROS_INFO("Publishing path for display");
  display_trajectory_publisher.publish(display_trajectory);
  joint_state_monitor.stop();
  ros::shutdown();
  spin_thread.join();
  return(0);

}

