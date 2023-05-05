/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2023, Jonathan Cacace <jonathan.cacace@gmail.com>
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "planner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;


bool PATH_PLANNER::isStateValid(const ob::State *state) {
    
    // cast the abstract state type to the type we expect
	const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

	// extract the first component of the state and cast it to what we expect
	const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	// extract the second component of the state and cast it to what we expect
	const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
        
    fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
	fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
	  
	fcl::CollisionObject treeObj((_tree_obj));
	fcl::CollisionObject robotObject(_Robot);

	
	robotObject.setTransform(rotation, translation);
	fcl::CollisionRequest requestType(1,false,1,false);
	fcl::CollisionResult collisionResult;
	fcl::collide(&robotObject, &treeObj, requestType, collisionResult);

	return(!collisionResult.isCollision());
    
}

/** Return an optimization objective which attempts to steer the robot
    away from obstacles. */
ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si) {
    return std::make_shared<ClearanceObjective>(si);
}


ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si) {
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto clearObj(std::make_shared<ClearanceObjective>(si));

    return 10.0*lengthObj + clearObj;
}

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si) {
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si) {
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
	return obj;
}


PATH_PLANNER::PATH_PLANNER(double xbounds[2], double ybounds[2], double zbounds[2]) {
  
    std::cout << "---Initializing OMPL path planner---" << std::endl;
	_space = ob::StateSpacePtr(new ob::SE3StateSpace());

	ob::RealVectorBounds bounds(3);

	bounds.setLow (0,  xbounds[0]);
	bounds.setHigh(0,  xbounds[1]); //x
	bounds.setLow (1,  ybounds[0]);
	bounds.setHigh(1,  ybounds[1]); //y
	bounds.setLow (2,  zbounds[0]);
	bounds.setHigh(2,  zbounds[1]); //z

	_space->as<ob::SE3StateSpace>()->setBounds(bounds);
	
    // construct an instance of  space information from this state space
	_si = ob::SpaceInformationPtr(new ob::SpaceInformation(_space));
	_si->setStateValidityChecker(std::bind(&PATH_PLANNER::isStateValid, this, std::placeholders::_1 ));
	_pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(_si));    
	_pdef->setOptimizationObjective(getPathLengthObjWithCostToGo(_si));
    //_pdef->setOptimizationObjective( getClearanceObjective(_si) );
    //_pdef->setOptimizationObjective( getThresholdPathLengthObj( _si ));

    _planner = ob::PlannerPtr(new og::RRTstar(_si));
	_Robot = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.3, 0.3, 0.1));

	std::cout << "---Planner initialized!---" << std::endl;
}



int PATH_PLANNER::plan(double max_t, std::vector<POSE> & poses) {

    //Planner not correctly initialized
    if( !_start_state_set || !_goal_state_set ) return -1;
        
    ob::ScopedState<ob::SE3StateSpace> start(_space);		
    ob::ScopedState<ob::SE3StateSpace> goal(_space);
    
    if( _random_start_state )
        start.random();
    else {
	    start->setXYZ(_start.position.x, _start.position.y, _start.position.z);
        start->rotation().w = _start.orientation.w;
        start->rotation().x = _start.orientation.x;
        start->rotation().y = _start.orientation.y;
        start->rotation().z = _start.orientation.z;
    }

    if( _random_goal_state )
        goal.random();
    else {
        goal->setXYZ(_goal.position.x, _goal.position.y, _goal.position.z);
        goal->rotation().w = _goal.orientation.w;
        goal->rotation().x = _goal.orientation.x;
        goal->rotation().y = _goal.orientation.y;
        goal->rotation().z = _goal.orientation.z;
    }  
    
    _pdef->clearGoal();
    _pdef->clearStartStates();
    _pdef->clearSolutionPaths();
    
    _pdef->setStartAndGoalStates(start, goal);
    
    _planner->clear();
    
    _planner->setProblemDefinition(_pdef);
    _planner->setup();

    if( _verbose ) _si->printSettings(std::cout);
	if( _verbose ) _pdef->print(std::cout);
    
    ob::PlannerStatus solved = _planner->solve(max_t);

    if( solved ) {
    
        ob::PathPtr path = _pdef->getSolutionPath();
        og::PathGeometric* pth = _pdef->getSolutionPath()->as<og::PathGeometric>();
        
        if(_verbose) pth->printAsMatrix(std::cout);
        
        poses.resize( pth->getStateCount () );

        for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++) {
		

        	const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();
			// extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
			// extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);


            poses[path_idx].position.x = pos->values[0];
            poses[path_idx].position.y = pos->values[1];
            poses[path_idx].position.z = pos->values[2];

            poses[path_idx].orientation.x = rot->x;
            poses[path_idx].orientation.y = rot->y;
            poses[path_idx].orientation.z = rot->z;
            poses[path_idx].orientation.w = rot->w;		
		}
    }
    reset_planner();

    //Return -1: wrong settings, 1: planned succeded, 0: no plan found
    return (solved) ? 1 : 0;
    
}