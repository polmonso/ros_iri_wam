#include <ros/ros.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <arm_navigation_msgs/utils.h>
#include <arm_navigation_msgs/OrientationConstraint.h>
#include <arm_navigation_msgs/PositionConstraint.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
class OmplTesting
{
	public:
	
	OmplTesting():nh_public("/")
	{

	}
	~OmplTesting(){}
	void getTrajectory(int argc, char** argv,geometry_msgs::PoseStamped& pose)
	{
	 if(argc < 8 || argc > 9) 
	 {
	   ROS_FATAL("Error: The numbers of parameters out of bound "); 
	   exit(1);
	 }
	 posicion.resize(3);
	 rotacion.resize(4);
	 for(int i =1; i <=7; ++i) 
	 {
	  (i < 4)? posicion[i-1]=strtod(argv[i],NULL):rotacion[i-4]=strtod(argv[i],NULL);
     }
     pose.pose.position.x=posicion[0];
     pose.pose.position.y=posicion[1];
     pose.pose.position.z=posicion[2];
     pose.pose.orientation.x=rotacion[0];
     pose.pose.orientation.y=rotacion[1];
     pose.pose.orientation.z=rotacion[2];
     pose.pose.orientation.w=rotacion[3];
     pose.header.frame_id="wam_fk/wam0";
     //pose.header.frame_id="world";
     pose.header.stamp=ros::Time::now();
     
	 time_move= (argc == 9)?ros::Duration(strtod(argv[8],NULL)):ros::Duration(3.0);
	} 
	
	void getMsg(arm_navigation_msgs::GetMotionPlan::Request& request,const geometry_msgs::PoseStamped& pose)
	{
	 request.motion_plan_request.group_name = "iri_wam";
     request.motion_plan_request.num_planning_attempts = 1;
     request.motion_plan_request.planner_id = std::string("");
     request.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
     arm_navigation_msgs::PositionConstraint pc;
     arm_navigation_msgs::OrientationConstraint oc;
     std::string link="wam_fk/wam7";
     arm_navigation_msgs::poseStampedToPositionOrientationConstraints(pose,link,pc,oc);
     request.motion_plan_request.goal_constraints.position_constraints.push_back(pc);
	 request.motion_plan_request.goal_constraints.orientation_constraints.push_back(oc);
	}
	void sendMsg_recieveResponse(const arm_navigation_msgs::GetMotionPlan::Request& msg, arm_navigation_msgs::GetMotionPlan::Response& response)
	{
		sendPlanningSceneMsg();
     ros::service::waitForService("/ompl_planning/plan_kinematic_path");	
	 ros::ServiceClient service=nh_public.serviceClient<arm_navigation_msgs::GetMotionPlan> ("/ompl_planning/plan_kinematic_path");	
     if(service.call(msg,response)){
     if(response.error_code.val != response.error_code.SUCCESS)
     {
      ROS_ERROR_STREAM("Motion planning failed "<<response.error_code);
     }
     else
     {
      ROS_INFO_STREAM("Motion planning succeeded");
     }
	}else ROS_ERROR("FAILED IN CALL");
    }
    bool sendPlanningSceneMsg()
    {
		arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
		arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
		ros::ServiceClient set_planning_scene_diff_client_ = nh_public.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>("/environment_server/set_planning_scene_diff");
		    if(!set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res)) {
      ROS_WARN("Can't get planning scene");
      return false;
      }
      return true;
	}
	void print(const arm_navigation_msgs::GetMotionPlan::Response& response)
	{
		ROS_INFO_STREAM(""<<response.robot_state);
		ROS_INFO_STREAM("#######################");
		ROS_INFO_STREAM(""<<response.trajectory);
	}
	private:
	
	
	ros::NodeHandle nh_public;
	ros::NodeHandle nh_private;
	std::vector<double> posicion;
	std::vector<double> rotacion;
	ros::Duration time_move;
	
	
	
};

int main(int argc,char ** argv)
{
	  ros::init(argc, argv, "ompl_node_testing");
	OmplTesting ompl;
	geometry_msgs::PoseStamped pose;
	arm_navigation_msgs::GetMotionPlan::Request request;
	arm_navigation_msgs::GetMotionPlan::Response response;
	
	ompl.getTrajectory(argc,argv,pose);
	ompl.getMsg(request,pose);
	ompl.sendMsg_recieveResponse(request,response);
	ompl.print(response);
	ros::spin();
	return 0;
}
