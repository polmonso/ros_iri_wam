#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
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

            //ompl_base::ScopedState<> start(space);
    //start.random();
            //ompl_base::ScopedState<> goal(space);
    //goal.random();
        //// set the start and goal states
    //simple_setup->setStartAndGoalStates(start, goal);

    //// this call is optional, but we put it in to get more output information
    //simple_setup->setup();
    //simple_setup->print();

    //// attempt to solve the problem within one second of planning time
    //bool solved = simple_setup->solve(1.0);

    //if (solved)
    //{
        //std::cout << "Found solution:" << std::endl;
        //// print the path to screen
        //simple_setup->simplifySolution();
        //simple_setup->getSolutionPath().print(std::cout);
    //}
    //else
        //std::cout << "No solution found" << std::endl;
        
	}
	~OMPL_3D()
	{
		delete bounds_;
		delete simple_setup_;
	}
	void initOmplStructures()
	{
		ompl_base::StateSpacePtr sp(new ompl_base::SE3StateSpace());
		space_=sp;
		ompl_base::RealVectorBounds vb(3);
		vb.setLow(-2);
		vb.setHigh(2);
		bounds_=&vb;
		space_->as<ompl_base::SE3StateSpace>()->setBounds(vb);
// hay 2 subespacios un RealVectorSpace representando traslacion y un SO3 representacion rotacion		
//		(space_->as<ompl_base::SE3StateSpace>())->getSubSpaceCount(); 0 ->t 1 -> qâ

	//	(space_->as<ompl_base::SE3StateSpace>())->getSubSpace(1).setAxisAngle(0.0,0.0,0.0,1.0);

       std::vector< ompl_base::StateSpacePtr > v=(space_->as<ompl_base::SE3StateSpace>())->getSubSpaces();
       //->setAxisAngle(0.0,0.0,0.0,1.0);
      ompl_base::StateSpacePtr pt= v[1];
      ompl_base::SO3StateSpace::StateType str;
      std::cout<<pt->getName()<<std::endl;
      pt::StateType.w=0.0;
		ompl_geometric::SimpleSetup ss(space_);
		simple_setup_=&ss;
	}
	void initCallbacks()
	 {
		simple_setup_->setStateValidityChecker(boost::bind(&isStateValid, _1));
        pose_listener_ = root_handle_.subscribe("pose_state", 1, &OMPL_3D::poseCB, this);
	 }
	 

	
	
	private:
	

	ompl_base::StateSpacePtr space_;
	ompl_base::RealVectorBounds *bounds_;
	ompl_geometric::SimpleSetup *simple_setup_;
	
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
	}

	
};
int main(int argc,char **argv)
{
	ros::init(argc, argv, "ompl_node_testing");
	OMPL_3D ompl;
	ros::spin();
	return 0;
}  
