#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "get_ik");
  ros::NodeHandle rh;
  static std::string info="/iri_wam_iri_wam_kinematics/get_ik_solver_info";
  static std::string ik="/iri_wam_iri_wam_kinematics/get_ik";
  ros::service::waitForService(ik);
  ros::service::waitForService(info);
  

  ros::ServiceClient ik_client = rh.serviceClient<kinematics_msgs::GetPositionIK>(ik);
  ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>(info);


  // define the service messages
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;

  if(query_client.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_DEBUG("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(1);
  }
  // define the service messages
  
  /*****************************************************
   * 
   * CAMBIAR LOS ARGV[I], POR LOS DATOS ENTREGADOS POR EL SERVICIO
   * 
   * REVISAR EL FRAME_ID
   * ****************************************************/
  kinematics_msgs::GetPositionIK::Request  gpik_req;
  kinematics_msgs::GetPositionIK::Response gpik_res;
  gpik_req.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = "wam_fk/wam7";
  gpik_req.ik_request.pose_stamped.header.frame_id = "wam_fk/wam0";
  gpik_req.ik_request.pose_stamped.pose.position.x = strtod(argv[1],NULL);
  gpik_req.ik_request.pose_stamped.pose.position.y = strtod(argv[2],NULL);
  gpik_req.ik_request.pose_stamped.pose.position.z = strtod(argv[3],NULL);

  gpik_req.ik_request.pose_stamped.pose.orientation.x = strtod(argv[4],NULL);
  gpik_req.ik_request.pose_stamped.pose.orientation.y = strtod(argv[5],NULL);
  gpik_req.ik_request.pose_stamped.pose.orientation.z = strtod(argv[6],NULL);
  gpik_req.ik_request.pose_stamped.pose.orientation.w = strtod(argv[7],NULL);
  
  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  
  gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
  
  
  for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
  {
    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
  }
  if(ik_client.call(gpik_req, gpik_res))
  {
    if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
      for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++)
        ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
    else
      ROS_ERROR("Inverse kinematics failed");
  }
  else
    ROS_ERROR("Inverse kinematics service call failed");
    
 
  arm_navigation_msgs::MoveArmGoal goalA;
  goalA.motion_plan_request.group_name="iri_wam";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
    
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name=std::string("ompl_planning/plan_kinematic_path");
  goalA.motion_plan_request.goal_constraints.joint_constraints.resize(gpik_res.solution.joint_state.name.size());
  double dt;
  for (unsigned int i = 0 ; i <goalA.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    goalA.motion_plan_request.goal_constraints.joint_constraints[i].joint_name =gpik_res.solution.joint_state.name[i];
    goalA.motion_plan_request.goal_constraints.joint_constraints[i].position =gpik_res.solution.joint_state.position[i];
    goalA.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below= gpik_res.solution.joint_state.position[i] -0.1;
    goalA.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above= gpik_res.solution.joint_state.position[i] +0.1;
    if(strtod(argv[i+1],NULL) == 0.0)
    {
     goalA.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below*= -1;
    }
    if(strtod(argv[i+1],NULL) < 0.0)
    {
     goalA.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above*= -1;
     goalA.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below*= -1;
    }
    goalA.motion_plan_request.path_constraints.joint_constraints[i].joint_name =gpik_res.solution.joint_state.name[i];
    goalA.motion_plan_request.path_constraints.joint_constraints[i].position =0.00; 
    goalA.motion_plan_request.path_constraints.joint_constraints[i].tolerance_below= 100;
    goalA.motion_plan_request.path_constraints.joint_constraints[i].tolerance_above= 100;
    // goalA.motion_plan_request.goal_constraints.joint_constraints[i].weight = 1.0;
  } 
  
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_iri_wam",true);

  move_arm.waitForServer();
  ROS_INFO("Connected to server");
      if (rh.ok())
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
