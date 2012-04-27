#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <stdio.h>

namespace cartesian_planning
{
 namespace ob = ompl::base;
 namespace og = ompl::geometric;
class CartesianPlanAlg
{
	public:
		CartesianPlanAlg();
		~CartesianPlanAlg();
	    bool planWithSimpleSetup(const ob::ScopedState<ompl::base::SE3StateSpace>& start,const ob::ScopedState<ompl::base::SE3StateSpace>& goal, const int& st, og::PathGeometric* ptrPath);
	    void makeOmplState(std::string estado,ob::ScopedState<ompl::base::SE3StateSpace>& state);
	    void writeFile(og::PathGeometric& path);
	    static bool isStateValid(const ob::State *state);
	    ob::StateSpacePtr getSpace() const;
	    void init();
	private:
		ob::StateSpacePtr space;
};
}
