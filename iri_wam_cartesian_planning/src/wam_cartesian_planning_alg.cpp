#include "wam_cartesian_planning_alg.h"
namespace ob = ompl::base;
namespace og = ompl::geometric;
WamCartesianPlanningAlgorithm::WamCartesianPlanningAlgorithm(void)
{
 // construct the state space we are planning in
 space.reset(new ob::SE3StateSpace());
 // set the bounds for the R^3 part of SE(3)
 ob::RealVectorBounds bounds(3);
 bounds.setLow(-2);
 bounds.setHigh(2);
 space->as<ob::SE3StateSpace>()->setBounds(bounds);
}

WamCartesianPlanningAlgorithm::~WamCartesianPlanningAlgorithm(void)
{
}

void WamCartesianPlanningAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

bool WamCartesianPlanningAlgorithm::isStateValid(const ob::State *state)
{
	// cast the abstract state type to the type we expect
	const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

	// extract the first component of the state and cast it to what we expect
	const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	// extract the second component of the state and cast it to what we expect
	const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

	// check validity of state defined by pos & rot


	// return a value that is always true but uses the two variables we define, so we avoid compiler warnings
	return (void*)rot != (void*)pos;
}

void WamCartesianPlanningAlgorithm::planWithSimpleSetup(ob::ScopedState<ompl::base::SE3StateSpace>& start,ob::ScopedState<ompl::base::SE3StateSpace>& goal, const int& st,std::vector<geometry_msgs::PoseStamped>& vector)
{
	// define a simple setup class
	og::SimpleSetup ss(space);
	// set state validity checking for this space
	ss.setStateValidityChecker(boost::bind(&isStateValid, _1));
	// create a random start state
	std::vector<geometry_msgs::PoseStamped> poses;
	// set the start and goal states
	ss.setStartAndGoalStates(start, goal);
	// this call is optional, but we put it in to get more output information
	ss.setup();
	ss.print();
	// attempt to solve the problem within one second of planning time
	bool solved = ss.solve(1.0);
	if (solved)
	{
		std::cout << "Found solution:" << std::endl;
		// print the path to screen
		ss.simplifySolution();
		og::PathGeometric path=ss.getSolutionPath();
		path.interpolate(st);
		omplToRos(path,poses);
/*		std::cout<<"SIZE "<<path.states.size()<<std::endl;
		path.print(std::cout);
		std::string pt="/home/irojas/Desktop/trajectories.txt";
		writeFile(path,pt);
*/ 
	}

	else std::cout << "No solution found" << std::endl;
}

void WamCartesianPlanningAlgorithm::omplToRos(og::PathGeometric& path, std::vector<geometry_msgs::PoseStamped>& vect)
{	
		geometry_msgs::PoseStamped msg;
		for(size_t i=0; i <path.getStates().size(); ++i)
		{
			msg.pose.position.x=path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getX();
			msg.pose.position.y=path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getY();
			msg.pose.position.z=path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getZ();
			msg.pose.orientation.x=path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().x;
			msg.pose.orientation.y=path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().y;
			msg.pose.orientation.z=path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().z;
			msg.pose.orientation.w=path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().w;
			vect.push_back(msg);
		}
}
void WamCartesianPlanningAlgorithm::writeFile(og::PathGeometric& path,std::string& pathDir)
{
	std::ofstream traj;
	traj.open(pathDir.c_str(), std::ofstream::out);
	float tx,ty,tz,qx,qy,qz,qw;
	for(size_t i=0; i <path.getStates().size(); ++i)
	{
		tx=path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getX();
		ty=path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getY();
		tz=path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getZ();
		qx=path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().x;
		qy=path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().y;
		qz=path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().z;
		qw=path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().w;
		
		traj<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<"\n";
		std::cout<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
	}
	traj.close();
}
