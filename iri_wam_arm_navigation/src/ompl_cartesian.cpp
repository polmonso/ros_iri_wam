#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Path.h>
#include <ompl/config.h>

#include "iri_wam_arm_navigation/PosePath.h"

#include <iostream>
/********************************************************************************************************
 * 
 * DESCRIPTION: THIS SOURCE CODE CALCULATE POINTS OF TRAJECTORY IN 3D COORDENATES (CARTESIAN COORDINATES)
 * 
 * STATUS: BETA
 * 
 * License: LGPL
 * 
 * AUTHOR: Ivan Rojas Jofre, irojas@iri.upc.edu
 * 
 *						  I.R.I
 * 		 Institut Robotica i Informatica Industrial
 * 		 	Universitat Politecnica Catalunya
 *		 				Barcelona
 * 						 España
 *********************************************************************************************************/

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
 	    initServices();  
 	    initPlanners();  
	}
	~OMPL_3D()
	{
		delete bounds_;
		delete simple_setup_;
	}
	/**
	 * Initialize OMPL Data Structures
	 */ 
	void initOmplStructures()
	{
		initStateSpace();
		simple_setup_=new ompl_geometric::SimpleSetup (space_);
		current_pose_ompl_= new ompl::base::ScopedState<ompl::base::SE3StateSpace>(space_);
		goal_pose_ompl_= new ompl::base::ScopedState<ompl::base::SE3StateSpace>(space_);		
	}
	
	/**
	 * Initialize ROS Service
	 */ 	
	void initServices()
	{
		pose_service_=root_handle_.advertiseService("pose_path",&OMPL_3D::computeCB,this);
	}
	/**
	 * Initialize CallBacks
	 */ 	
	void initCallbacks()
	{
		simple_setup_->setStateValidityChecker(boost::bind(&isStateValid, _1));
        pose_listener_ = root_handle_.subscribe("pose_state", 1, &OMPL_3D::poseCB, this);
	}	
	/**
	 * Initialize Ompl Planners
	 */ 
	void initPlanners()
	{
	 ompl_base::PlannerPtr planner(new ompl_geometric::SBL(simple_setup_->getSpaceInformation()));
	 planner->as<ompl_geometric::SBL>()->setRange(0.001);
  	 simple_setup_->setPlanner(planner);		
	}
	private:
	
	ompl_base::StateSpacePtr space_;
	ompl_base::RealVectorBounds *bounds_;
	ompl_geometric::SimpleSetup *simple_setup_;
	/**
	 * This parameter save the actual state of robot. Who refresh this information is the method "poseCB" each time it receives the topic "pose_state",this is a callback
	 */ 
	ompl::base::ScopedState<ompl::base::SE3StateSpace> * current_pose_ompl_;
	/**
	 * This parameter contain the goal pose of robot. Who refresh this information is the method computeCB.
	 */ 	
	ompl::base::ScopedState<ompl::base::SE3StateSpace> * goal_pose_ompl_;
	
	ros::NodeHandle root_handle_;
	ros::NodeHandle private_handle_;
	ros::Subscriber pose_listener_;
	ros::ServiceServer pose_service_;
	/**
	 * This parameter contain the current position (in 3D) of robot in ROS format
	 */ 
	geometry_msgs::PoseStamped current_pose_;
	
	bool pose_recieve_;
	/**
	 * This method return true if state is valid, false in other case
	 */
	static bool isStateValid(const ompl_base::State *state)
	{
		const ompl_base::SE3StateSpace::StateType *se3state = state->as<ompl_base::SE3StateSpace::StateType>();
		const ompl_base::RealVectorStateSpace::StateType *pos = se3state->as<ompl_base::RealVectorStateSpace::StateType>(0);
		const ompl_base::SO3StateSpace::StateType *rot = se3state->as<ompl_base::SO3StateSpace::StateType>(1);
		return (void*)rot != (void*)pos;
	}
	/**
	 * This method receives the current position (3D) for robot, This is a topic of ROS
	 */ 
	void poseCB(const geometry_msgs::PoseStamped::ConstPtr& current_pose)
	{
		current_pose_=*current_pose;
		ros_to_ompl(current_pose_);
		pose_recieve_=true;
	}
	/**
	 * This method calculate a trajectory between two points in cartesian coordinates.
	 * The trajectory contain points in cartesian coordenates.
	 */ 
	bool computeCB(iri_wam_arm_navigation::PosePath::Request& req,iri_wam_arm_navigation::PosePath::Response& resp)
	{
		geometry_msgs::PoseStamped msg=req.pose_stamped;
		(*goal_pose_ompl_)->setXYZ(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
		(*goal_pose_ompl_)->rotation().x= msg.pose.orientation.x;
		(*goal_pose_ompl_)->rotation().y= msg.pose.orientation.y;
		(*goal_pose_ompl_)->rotation().z= msg.pose.orientation.z;
		(*goal_pose_ompl_)->rotation().w= msg.pose.orientation.w;
		
	 	 simple_setup_->setStartAndGoalStates(*current_pose_ompl_, *goal_pose_ompl_);
		simple_setup_->setup();
		simple_setup_->print();	
		bool solved = simple_setup_->solve(2.0);
		if (solved & pose_recieve_)
		{
         std::cout << "Found solution:" << std::endl;
         // print the path to screen
         simple_setup_->simplifySolution();
         OmplPathToRosPath(simple_setup_->getSolutionPath().states,resp);
		}
		else if(not solved)  ROS_INFO_STREAM("No solution found" << std::endl);
		else
		{
			ROS_FATAL("NO POSE RECIEVE FROM ROBOT");
			exit(1);
		}
			return true;
	}
	/**
	 * Convert information of ROS format to OMPL format
	 */ 
	void ros_to_ompl(geometry_msgs::PoseStamped& msg)
	{
		(*current_pose_ompl_)->setXYZ(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
		(*current_pose_ompl_)->rotation().x= msg.pose.orientation.x;
		(*current_pose_ompl_)->rotation().y= msg.pose.orientation.y;
		(*current_pose_ompl_)->rotation().z= msg.pose.orientation.z;
		(*current_pose_ompl_)->rotation().w= msg.pose.orientation.w;
	}
	/**
	 * Convert information of OMPL format to ROS format
	 */ 	
	void OmplPathToRosPath(std::vector<ompl_base::State*> vector,iri_wam_arm_navigation::PosePath::Response& resp)
	{
		geometry_msgs::PoseStamped tmp;	
		for(size_t i=0; i < vector.size(); ++i)
		{
			getPose(vector[i],tmp);
			resp.pose_stamped.push_back(tmp);
		}
	}
	/**
	 *  Convert information of OMPL format to ROS format
	 */ 
	void getPose( ompl_base::State* state, geometry_msgs::PoseStamped& p)
	{
	     p.pose.position.x= state->as<ompl::base::SE3StateSpace::StateType>()->getX();
         p.pose.position.y= state->as<ompl::base::SE3StateSpace::StateType>()->getY();
         p.pose.position.z= state->as<ompl::base::SE3StateSpace::StateType>()->getZ();
         p.pose.orientation.x= state->as<ompl::base::SE3StateSpace::StateType>()->rotation().x;
         p.pose.orientation.y= state->as<ompl::base::SE3StateSpace::StateType>()->rotation().y;
         p.pose.orientation.z= state->as<ompl::base::SE3StateSpace::StateType>()->rotation().z;
         p.pose.orientation.w= state->as<ompl::base::SE3StateSpace::StateType>()->rotation().w;		
	}
	void initStateSpace()
	{

		space_.reset(new ompl_base::SE3StateSpace());		
		ompl_base::StateSpacePtr real_vector_state_space = space_->as<ompl_base::SE3StateSpace>()->getSubSpace("RealVectorSpace2");

		real_vector_state_space->as<ompl::base::RealVectorStateSpace>()->addDimension("x",-2.0,2.0);    
		real_vector_state_space->as<ompl::base::RealVectorStateSpace>()->addDimension("y",-2.0,2.0);    
		real_vector_state_space->as<ompl::base::RealVectorStateSpace>()->addDimension("z",-2.0,2.0);    
		real_vector_state_space->as<ompl::base::RealVectorStateSpace>()->addDimension("roll",-M_PI,M_PI);    
		real_vector_state_space->as<ompl::base::RealVectorStateSpace>()->addDimension("pith",-M_PI,M_PI);    
		real_vector_state_space->as<ompl::base::RealVectorStateSpace>()->addDimension("yaw",-M_PI,M_PI);    
		space_->setLongestValidSegmentFraction(0.0001);
	}
};
int main(int argc,char **argv)
{
	ros::init(argc, argv, "ompl_node_testing");
	OMPL_3D ompl;
	ros::spin();
	return 0;
}  

