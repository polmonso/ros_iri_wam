#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>


#include <iostream>

namespace ompl_base = ompl::base;
namespace ompl_geometric = ompl::geometric;

class OMPL_3D
{
	public:
	OMPL_3D():
	root_handle_("/"),
	private_handle_("~"),
	pose_recieve_(false)
	{
		initOmplStructures();
 	    initCallbacks();      
	}
	~OMPL_3D()
	{
		delete bounds_;
		delete simple_setup_;
	}
// hay 2 subespacios un RealVectorSpace representando traslacion y un SO3 representacion rotacion		
	void initOmplStructures()
	{
		ompl_base::StateSpacePtr sp(new ompl_base::SE3StateSpace());
		space_=sp;
		ompl_base::RealVectorBounds vb(3);
		vb.setLow(-2);
		vb.setHigh(2);
		bounds_=&vb;
		space_->as<ompl_base::SE3StateSpace>()->setBounds(vb);
		ompl::base::ScopedState<ompl::base::SE3StateSpace> state(space_);
		current_pose_ompl_= new ompl::base::ScopedState<ompl::base::SE3StateSpace>(space_);
		goal_pose_ompl_= new ompl::base::ScopedState<ompl::base::SE3StateSpace>(space_);
		ompl_geometric::SimpleSetup ss(space_);
		simple_setup_=new ompl_geometric::SimpleSetup (space_);
	}
	void initCallbacks()
	{
		simple_setup_->setStateValidityChecker(boost::bind(&isStateValid, _1));
        pose_listener_ = root_handle_.subscribe("pose_state", 1, &OMPL_3D::poseCB, this);
	}	
	void initPlanners()
	{
	  ompl_base::PlannerPtr planner(new ompl_geometric::SBL(simple_setup_->getSpaceInformation()));
  	  simple_setup_->setPlanner(planner);		
	}
	private:
	
	ompl_base::StateSpacePtr space_;
	ompl_base::RealVectorBounds *bounds_;
	ompl_geometric::SimpleSetup *simple_setup_;
	ompl::base::ScopedState<ompl::base::SE3StateSpace> * current_pose_ompl_;
	ompl::base::ScopedState<ompl::base::SE3StateSpace> * goal_pose_ompl_;
	
	ros::NodeHandle root_handle_;
	ros::NodeHandle private_handle_;
	ros::Subscriber pose_listener_;
	ros::ServiceServer pose_service_;
	
	geometry_msgs::PoseStamped current_pose_;
	
	bool pose_recieve_;
	
	static bool isStateValid(const ompl_base::State *state)
	{
		const ompl_base::SE3StateSpace::StateType *se3state = state->as<ompl_base::SE3StateSpace::StateType>();
		const ompl_base::RealVectorStateSpace::StateType *pos = se3state->as<ompl_base::RealVectorStateSpace::StateType>(0);
		const ompl_base::SO3StateSpace::StateType *rot = se3state->as<ompl_base::SO3StateSpace::StateType>(1);
		return (void*)rot != (void*)pos;
	}
	void poseCB(const geometry_msgs::PoseStamped::ConstPtr& current_pose)
	{
		current_pose_=*current_pose;
		pose_recieve_=true;
	}
	void computeCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		(*goal_pose_ompl_)->setXYZ(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
		(*goal_pose_ompl_)->rotation().setAxisAngle(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);	
	 	 simple_setup_->setStartAndGoalStates(*current_pose_ompl_, *goal_pose_ompl_);
		simple_setup_->setup();
		simple_setup_->print();	
		bool solved = simple_setup_->solve(1.0);
		if (solved)
		{
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        simple_setup_->simplifySolution();
        simple_setup_->getSolutionPath().print(std::cout);
		}
		else
        std::cout << "No solution found" << std::endl;
		
		}
	void ros_to_ompl(geometry_msgs::PoseStamped& msg)
	{
		(*current_pose_ompl_)->setXYZ(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
		(*current_pose_ompl_)->rotation().setAxisAngle(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);
	}

	
};
int main(int argc,char **argv)
{
	ros::init(argc, argv, "ompl_node_testing");
	OMPL_3D ompl;
	ros::spin();
	return 0;
}  
