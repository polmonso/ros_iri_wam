#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;

class PlanRigidBody
{
	public:
	PlanRigidBody()
	{
	// construct the state space we are planning in
	space.reset(new ob::SE3StateSpace());

	// set the bounds for the R^3 part of SE(3)
	ob::RealVectorBounds bounds(3);
	bounds.setLow(-2);
	bounds.setHigh(2);

	space->as<ob::SE3StateSpace>()->setBounds(bounds);
 int real_vector_index = space->as<ompl::base::CompoundStateSpace>()->getSubSpaceCount();
std::cout<<"real vector:= "<<real_vector_index<<std::endl;
	}
	
	~PlanRigidBody(){}
	
	static bool isStateValid(const ob::State *state)
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

	void planWithSimpleSetup(ob::ScopedState<ompl::base::SE3StateSpace>& start,ob::ScopedState<ompl::base::SE3StateSpace>& goal, int st)
	{
	// define a simple setup class
	og::SimpleSetup ss(space);
	// set state validity checking for this space
	ss.setStateValidityChecker(boost::bind(&isStateValid, _1));
	// create a random start state

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
		//ss.getSolutionPath().print(std::cout);
		og::PathGeometric path=ss.getSolutionPath();
		//std::cout<<"SIZE "<<path.states_<<std::endl;
		path.interpolate(st);
		std::cout<<"SIZE "<<path.states.size()<<std::endl;
		path.print(std::cout);
		writeFile(path);
	}
	else
		std::cout << "No solution found" << std::endl;
	}


	void getKeyboard(std::string estado,ob::ScopedState<ompl::base::SE3StateSpace>& state)
	{
		std::cout<<"Ingresa el "<<estado<<std::endl;
		float tx,ty,tz,qx,qy,qz,qw;
		std::cout<<"Ingresa tx (traslacion en X) "<<std::endl;
		std::cin>>tx;
		std::cout<<"tx ingresado "<<tx<<std::endl;
		std::cout<<"Ingresa ty (traslacion en Y) "<<std::endl;
		std::cin>>ty;
		std::cout<<"ty ingresado "<<ty<<std::endl;
		std::cout<<"Ingresa tz (traslacion en Z) "<<std::endl;
		std::cin>>tz;
		std::cout<<"tz ingresado "<<tz<<std::endl;
		std::cout<<"Ingresa qx (Rotacion en X,Quaternion) "<<std::endl;
		std::cin>>qx;
		std::cout<<"qx ingresado "<<qx<<std::endl;
		std::cout<<"Ingresa qy (Rotacion en Y,Quaternion) "<<std::endl;
		std::cin>>qy;
		std::cout<<"qy ingresado "<<qy<<std::endl;
		std::cout<<"Ingresa qz (Rotacion en Z,Quaternion) "<<std::endl;
		std::cin>>qz;
		std::cout<<"qz ingresado "<<qz<<std::endl;
		std::cout<<"Ingresa qw (Angulo del Quaternion) "<<std::endl;
		std::cin>>qw;
		std::cout<<"qw ingresado "<<qw<<std::endl;
		state->setXYZ(tx,ty,tz);
		state->rotation().x=qx;
		state->rotation().y=qy;
		state->rotation().z=qz;
		state->rotation().w=qw;
		state.print();		
	}
	void writeFile(og::PathGeometric& path)
	{
		std::ofstream traj;
		traj.open("/home/irojas/Desktop/trajectories.txt", std::ofstream::out);
		float tx,ty,tz,qx,qy,qz,qw;
		for(size_t i=0; i <path.states.size(); ++i)
		{
			tx=path.states[i]->as<ob::SE3StateSpace::StateType>()->getX();
			ty=path.states[i]->as<ob::SE3StateSpace::StateType>()->getY();
			tz=path.states[i]->as<ob::SE3StateSpace::StateType>()->getZ();
			qx=path.states[i]->as<ob::SE3StateSpace::StateType>()->rotation().x;
			qy=path.states[i]->as<ob::SE3StateSpace::StateType>()->rotation().y;
			qz=path.states[i]->as<ob::SE3StateSpace::StateType>()->rotation().z;
			qw=path.states[i]->as<ob::SE3StateSpace::StateType>()->rotation().w;
			
			traj<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<"\n";
			std::cout<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
		}
		traj.close();
	}
	ob::StateSpacePtr space;
};

int main(int argc, char ** argv)
{
	PlanRigidBody plan;
	int st=0;
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    ob::ScopedState<ompl::base::SE3StateSpace> goal(plan.space);
    ob::ScopedState<ompl::base::SE3StateSpace> start(plan.space);
	//plan.getTrajectory(argc,argv,goal);
	plan.getKeyboard("Estado Inicial",start);
getchar();
   // std::cout << std::endl << std::endl;
	plan.getKeyboard("Estado Final",goal);
getchar();
		std::cout<<"Ingresa Cantidad de estados deseados(puntos de la trayectorias) "<<std::endl;
		std::cin>>st;
		std::cout<<"cantidad de estados ingresado "<<st<<std::endl;		
    plan.planWithSimpleSetup(start,goal,st);

    return 0;
}
