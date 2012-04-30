#include "iri_wam_arm_navigation/cartesian_planning_alg_node.h"

using namespace cartesian_planning;

CartesianPlanAlgNode::CartesianPlanAlgNode():
	root_handle_("/"),
	private_handle_("~")
	{
		current_pos_sub=root_handle_.subscribe("current_pose",1,&CartesianPlanAlgNode::current_pos_CB,this);
		alg_.init();
		goal= new ob::ScopedState<ompl::base::SE3StateSpace>(alg_.getSpace());
		current= new ob::ScopedState<ompl::base::SE3StateSpace>(alg_.getSpace());
		solution_service=root_handle_.advertiseService("get_pose_path",&CartesianPlanAlgNode::getPath,this);
		private_handle_.param<int>("number_of_states", number_states, 10);

	}
CartesianPlanAlgNode::~CartesianPlanAlgNode()
{
}
 void CartesianPlanAlgNode::current_pos_CB(const geometry_msgs::Pose::ConstPtr& input)
 {
	RostoOmplState( *input ,current);
 }
 void CartesianPlanAlgNode::RostoOmplState(const geometry_msgs::Pose& input,ob::ScopedState<ompl::base::SE3StateSpace> *ptr)
 {
	ob::ScopedState<ompl::base::SE3StateSpace> tmp(alg_.getSpace());
	tmp->setXYZ(input.position.x,input.position.y,input.position.z);
	tmp->rotation().x=input.orientation.x;
	tmp->rotation().y=input.orientation.y;
	tmp->rotation().z=input.orientation.z;
	tmp->rotation().w=input.orientation.w;
	printNode(1,tmp);
	ptr= new ob::ScopedState<ompl::base::SE3StateSpace>(tmp);
 }
 //type = print type
 //0-> ROS_INFO
 //1 -> ROS_DEBUG
 //2-> ROS_WARN
 //3-> ROS_ERROR
 void CartesianPlanAlgNode::printNode(const int type,const ob::ScopedState<ompl::base::SE3StateSpace>& state)const
 {	 
	 if (type == 0) ROS_INFO_STREAM(""<<state);
	 else if (type == 1) ROS_DEBUG_STREAM(""<<state);
	 else if (type == 2) ROS_WARN_STREAM(""<<state);
	 else if (type == 3) ROS_ERROR_STREAM(""<<state);		 
 }
bool CartesianPlanAlgNode::getPath(iri_wam_arm_navigation::PosePath::Request &request, iri_wam_arm_navigation::PosePath::Response &response)
{
    lock.lock();
    og::PathGeometric* path=NULL;
    geometry_msgs::Pose pose= request.pose;
    RostoOmplState(pose,goal);
    printNode(1,*goal);
    printNode(2,*goal);
    printNode(2,*current);
    bool resp=alg_.planWithSimpleSetup(*current,*goal, number_states,path);
    if(resp)
    {
		std::vector<geometry_msgs::Pose> poses;
		OmpltoRosMsg(path,poses);
		response.solution_found=true;
		response.poses_solution=poses;
	}
	else 
	{
		response.solution_found=false;
	}
    lock.unlock();
	return true;
}
void CartesianPlanAlgNode::OmpltoRosMsg(og::PathGeometric* ptrPath,std::vector<geometry_msgs::Pose>& poses)
{
	og::PathGeometric path= *ptrPath;
	geometry_msgs::Pose pose;
	for(size_t i=0; i <path.states.size(); ++i)
	{		
		pose.position.x=path.states[i]->as<ob::SE3StateSpace::StateType>()->getX();
		pose.position.y=path.states[i]->as<ob::SE3StateSpace::StateType>()->getY();
		pose.position.z=path.states[i]->as<ob::SE3StateSpace::StateType>()->getZ();
		pose.orientation.x=path.states[i]->as<ob::SE3StateSpace::StateType>()->rotation().x;
		pose.orientation.y=path.states[i]->as<ob::SE3StateSpace::StateType>()->rotation().y;
		pose.orientation.z=path.states[i]->as<ob::SE3StateSpace::StateType>()->rotation().z;
		pose.orientation.w=path.states[i]->as<ob::SE3StateSpace::StateType>()->rotation().w;
		poses.push_back(pose);
	}
}
int main(int argc, char ** argv)
{
 ros::init(argc,argv,"Cartesian_Planning");
 CartesianPlanAlgNode alg;
 ros::spin();

return 0;
}
