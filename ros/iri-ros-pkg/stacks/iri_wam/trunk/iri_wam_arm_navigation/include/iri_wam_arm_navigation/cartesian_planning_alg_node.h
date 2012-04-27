#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "iri_wam_arm_navigation/PosePath.h"
#include "iri_wam_arm_navigation/cartesian_planning_alg.h"
namespace cartesian_planning
{
class CartesianPlanAlgNode
{

	public:
	
		CartesianPlanAlgNode();

		~CartesianPlanAlgNode();
		void printNode(const int type,const ob::ScopedState<ompl::base::SE3StateSpace>& state)const;

	private:
	
	
	 void current_pos_CB(const geometry_msgs::Pose::ConstPtr& input);
     void RostoOmplState(const geometry_msgs::Pose& input,ob::ScopedState<ompl::base::SE3StateSpace> *ptr);
	 bool getPath(iri_wam_arm_navigation::PosePath::Request &request, iri_wam_arm_navigation::PosePath::Response &response);
	 void OmpltoRosMsg(og::PathGeometric* ptrPath, std::vector<geometry_msgs::Pose>& poses );
	 
	 //[Nodes]
	 ros::NodeHandle root_handle_;
	 ros::NodeHandle private_handle_;
	 //[Subcriber]
	 ros::Subscriber current_pos_sub;
	 
	 //[Publisher]
	 ros::Publisher  diagnostics_pub;
	 //[Service]
	 ros::ServiceServer solution_service;
	 
     int number_states;
	 boost::mutex lock;
	 CartesianPlanAlg alg_;
	 ob::ScopedState<ompl::base::SE3StateSpace> *goal;
	 ob::ScopedState<ompl::base::SE3StateSpace> *current;
};
}
