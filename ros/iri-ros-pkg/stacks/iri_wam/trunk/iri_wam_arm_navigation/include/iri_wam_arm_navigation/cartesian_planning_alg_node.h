#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "iri_wam_arm_navigation/PosePath.h"
#include "iri_wam_arm_navigation/cartesian_planning_alg.h"

class CartesianPlanAlgNode
{

	public:
	
		CartesianPlanAlgNode();

		~CartesianPlanAlgNode();


	private:
	
	
	 void current_pos_CB(const geometry_msgs::Pose::ConstPtr& input);

	
	// void RosPoseOmplState()
	
	 ros::NodeHandle root_handle_;
	 ros::NodeHandle private_handle_;
	 //[Subcriber]
	 ros::Subscriber current_pos_sub;
	 ros::Subscriber current_pos_stamped_sub;
	 //[Publisher]
	 ros::Publisher  diagnostics_pub;
	 //[Service]
	 ros::ServiceServer solution_service;
	 
	 CartesianPlanAlg *alg_;

};
