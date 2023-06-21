
#include <stdio.h>
#include "boost/thread.hpp"

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/config.h>

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

class OMPL_PLAN {
	public:
		OMPL_PLAN();
		void run();
		void ompl_init();
		void plan();
		ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);
		bool isStateValid(const ob::State *state);

	private:
		ob::StateSpacePtr _space;
		ob::SpaceInformationPtr _si;
		ob::ProblemDefinitionPtr _pdef;
		//ros::Publisher _path_pub;
		//ros::NodeHandle _nh;	
};



OMPL_PLAN::OMPL_PLAN() {

}

bool OMPL_PLAN::isStateValid(const ob::State *state) {
	return true;
}

ob::OptimizationObjectivePtr OMPL_PLAN::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si) {
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
	return obj;
}

void OMPL_PLAN::ompl_init() {

	_space = ob::StateSpacePtr(new ob::SE3StateSpace());
	
	ob::RealVectorBounds bounds(3);
	bounds.setLow(0,-20);
	bounds.setHigh(0,20); //x
	bounds.setLow(1,-20);
	bounds.setHigh(1,20); //y
	bounds.setLow(2, 0);
	bounds.setHigh(2,3); //z
	_space->as<ob::SE3StateSpace>()->setBounds(bounds);
	
	// construct an instance of  space information from this state space
	_si = ob::SpaceInformationPtr(new ob::SpaceInformation(_space));
	
	ob::ScopedState<ob::SE3StateSpace> start(_space);		
	start->setXYZ(0,0,1);
	start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	
	ob::ScopedState<ob::SE3StateSpace> goal(_space);
	goal->setXYZ(10,2,1);
	goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

	_si->setStateValidityChecker(std::bind(&OMPL_PLAN::isStateValid, this, std::placeholders::_1 ));
	
	_pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(_si));
	_pdef->setStartAndGoalStates(start, goal);
	_pdef->setOptimizationObjective(OMPL_PLAN::getPathLengthObjWithCostToGo(_si));
	

	std::cout << "Planner initialized!" << std::endl;
}

void OMPL_PLAN::plan() {

	cout << "Press enter to plan!" << endl;
	string line;
	getline(cin, line);

	// create a planner for the defined space
	ob::PlannerPtr plan(new og::RRTstar(_si));

	// set the problem we are trying to solve for the planner
	plan->setProblemDefinition(_pdef);

	// perform setup steps for the planner
	plan->setup();

	// print the settings for this space
	_si->printSettings(std::cout);
	_pdef->print(std::cout);

	// attempt to solve the problem within one second of planning time
	ob::PlannerStatus solved = plan->solve(2);

	if (solved) {

		std::cout << "Found solution:" << std::endl;
		ob::PathPtr path = _pdef->getSolutionPath();
		og::PathGeometric* pth = _pdef->getSolutionPath()->as<og::PathGeometric>();
		pth->printAsMatrix(std::cout);

		/*
        nav_msgs::Path generated_path;
		generated_path.header.frame_id = "map";
		geometry_msgs::PoseStamped p;
		p.header.frame_id = "map";
        */
		
        for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++) {

			const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();
			// extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
			// extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

            /*
			p.pose.position.x = pos->values[0];
			p.pose.position.y = pos->values[1];
			p.pose.position.z = pos->values[2];

			p.pose.orientation.x = rot->x;
			p.pose.orientation.y = rot->y;
			p.pose.orientation.z = rot->z;
			p.pose.orientation.w = rot->w;

			generated_path.poses.push_back( p );
            */      
		}

		/*
        ros::Rate r(10);
		while(ros::ok()) {
			_path_pub.publish( generated_path );
			r.sleep();
		}
        */

	}
}

void OMPL_PLAN::run() {
	
    ompl_init();
	//boost::thread plan_t( &OMPL_PLAN::plan, this );
	plan();
    //ros::spin();

}

int main(int argc, char** argv) {

	//ros::init(argc, argv, "ompl_planning");
	OMPL_PLAN op;
	op.run();
	
    return 0;

}

