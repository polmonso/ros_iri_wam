#include "iri_wam_arm_navigation/cartesian_planning_alg_node.h"



CartesianPlanAlgNode::CartesianPlanAlgNode():
	root_handle_("/"),
	private_handle_("~")
	{
		current_pos_sub=root_handle_.subscribe("current_pose",1,&CartesianPlanAlgNode::current_pos_CB,this);
		alg_ = new CartesianPlanAlg(); 
	}
CartesianPlanAlgNode::~CartesianPlanAlgNode()
{
}
 void CartesianPlanAlgNode::current_pos_CB(const geometry_msgs::Pose::ConstPtr& input)
 {
 }

int main(int argc, char ** argv)
{


return 0;
}
