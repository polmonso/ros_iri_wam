#include <ros/ros.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <arm_navigation_msgs/utils.h>
#include <arm_navigation_msgs/OrientationConstraint.h>
#include <arm_navigation_msgs/PositionConstraint.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include "iri_wam_arm_navigation/PosePath.h"
class Ompl_Test_Cart
{
	public:
	Ompl_Test_Cart(){}
	~Ompl_Test_Cart(){}
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
     pose.header.frame_id="world";
     pose.header.stamp=ros::Time::now();
	} 
	void sendMsg(const geometry_msgs::PoseStamped& pose)
	{
		ros::service::waitForService("/pose_path");	
		ros::ServiceClient client=root_handle_.serviceClient<iri_wam_arm_navigation::PosePath> ("/pose_path");	
		iri_wam_arm_navigation::PosePath::Request req;
		iri_wam_arm_navigation::PosePath::Response resp;
		req.pose_stamped=pose;
		if(client.call(req,resp))
		{
			for(unsigned int i=0; i < resp.pose_stamped.size(); ++i){
			ROS_INFO_STREAM(""<<resp.pose_stamped[i]);
			ROS_INFO("INTRO TO CONTINUE");
			getchar();
			}
		}
		else
		{
		ROS_ERROR("[ompl_key] PosePath Service failure");
		}
		
	}
	
	private:
	ros::NodeHandle root_handle_;
	std::vector<double> posicion;
	std::vector<double> rotacion;

};
int main(int argc,char **argv)
{
	  ros::init(argc, argv, "ompl_node_testing");
	Ompl_Test_Cart ompl;
	geometry_msgs::PoseStamped pose;
	ompl.getTrajectory(argc,argv,pose);
	ompl.sendMsg(pose);
	ros::spin();
}
