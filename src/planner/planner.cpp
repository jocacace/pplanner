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
    
    if( _tree_obj ) {
        // cast the abstract state type to the type we expect
        const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
            
        fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
        fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
        
        //TODO: check here!
        fcl::CollisionObject treeObj((_tree_obj));
        fcl::CollisionObject robotObject(_Robot);

        
        robotObject.setTransform(rotation, translation);
        fcl::CollisionRequest requestType(1,false,1,false);
        fcl::CollisionResult collisionResult;
        fcl::collide(&robotObject, &treeObj, requestType, collisionResult);

        return(!collisionResult.isCollision());
    }
    else {        
        return true;
    }
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


PATH_PLANNER::PATH_PLANNER(double xbounds[2], double ybounds[2], double zbounds[2] ) {
  
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
	//_pdef->setOptimizationObjective(getPathLengthObjWithCostToGo(_si));
    _pdef->setOptimizationObjective( getClearanceObjective(_si) );
    //_pdef->setOptimizationObjective( getThresholdPathLengthObj( _si ));

    _planner = ob::PlannerPtr(new og::RRTstar(_si));    
	//_Robot = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(r_dim[0], r_dim[1], r_dim[2]));

	std::cout << "---Planner initialized!---" << std::endl;
}

int PATH_PLANNER::test_pruning(std::vector<POSE> & poses, std::vector<POSE> & opt_poses, std::vector<POSE> & checked_points ) {

    poses.resize(3);
    poses[0].position.x = 0.0;
    poses[0].position.y = 0.0;
    poses[0].position.z = 0.0;
    poses[0].orientation.w = 1.0;

    poses[1].position.x = 5.0;
    poses[1].position.y = 5.0;
    poses[1].position.z = 0.0;
    poses[1].orientation.w = 1.0;

    poses[2].position.x = 3.0;
    poses[2].position.y = 10.0;
    poses[2].position.z = 8.0;
    poses[2].orientation.w = 1.0;

    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
    Eigen::Vector3d pX;
    Eigen::Vector3d s;

    double delta = 0.3;

    //Test pruning from point 0->2
    p0 << poses[0].position.x, poses[0].position.y, poses[0].position.z;
    p1 << poses[2].position.x, poses[2].position.y, poses[2].position.z;
    pX = p0;

    s =  (p1-p0);
    s /= s.norm();

    std::cout << "s: " << s.transpose() << std::endl;
    POSE cp;

    while( (pX-p1).norm() > delta ) {

        

        pX += s*delta;

        cp.position.x = pX[0];
        cp.position.y = pX[1];
        cp.position.z = pX[2];

        checked_points.push_back( cp );

    }

    //checked_points

    return 1;

}



int PATH_PLANNER::optimize_path(std::vector<POSE> poses, std::vector<POSE> & opt_poses) {

    opt_poses = poses;
    if( poses.size() == 2 ) {
        std::cout << "[optimization not useful] only 2 points in the path" << std::endl;
        //opt_poses = poses;
        return 0;
    } //Two poitns 
    
    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
    Eigen::Vector3d pX;
    Eigen::Vector3d s;

    double delta = 0.2;

    int tries = 5; //just 5 optimization tries (consider elapsed time instead)
    int ct = 0;
  
    fcl::CollisionObject treeObj((_tree_obj));
    fcl::CollisionObject robotObject(_Robot);

    auto start = boost::chrono::steady_clock::now();

    bool elapsed_time = false;
    //while( !elapsed_time && opt_poses.size() > 2 ) {

    for ( int i=0; i<opt_poses.size()-1; i++ ) {
        for ( int j=i+1;j<opt_poses.size(); j++ ) {
        

        const int i0_range_from     = 0;
        const float i0_range_to     = opt_poses.size()-1;
        std::random_device          i0_rand_dev;
        std::mt19937                i0_generator(i0_rand_dev());
        std::uniform_int_distribution<int>  i0_distr(i0_range_from, i0_range_to);

        const int i1_range_from     =  0;
        const float i1_range_to     = opt_poses.size()-1;
        std::random_device          i1_rand_dev;
        std::mt19937                i1_generator(i1_rand_dev());
        std::uniform_int_distribution<int>  i1_distr(i1_range_from, i1_range_to);


        int i0;
        int i1;

        i0 = i0_distr(i0_generator);
        i1 = i1_distr(i1_generator);

        i0=i;
        i1=j;
        std::cout << "Point indeces: " <<  i0 << " " << i1 << std::endl;

        if( i0 == i1 ) {
            std::cout << "Numeri uguali" << std::endl;
            continue;
        }
        else if(  abs( i0-i1 ) == 1 ) {
            std::cout << "segmenti contigui" << std::endl;
            continue;
        }
        else {
            
            int ptemp;
            if( i1 < i0 ) {
                ptemp = i0;
                i0 = i1;
                i1 = ptemp;
            }        

            std::cout << "Size opt poses: " << opt_poses.size() << std::endl;

            p0 << opt_poses[i0].position.x, opt_poses[i0].position.y, opt_poses[i0].position.z;
            p1 << opt_poses[i1].position.x, opt_poses[i1].position.y, opt_poses[i1].position.z;
            
            pX = p0;

            bool obs = false;

            s =  (p1-p0);
            s /= s.norm();
            while( (pX-p1).norm() > delta && !obs ) {

                //std::cout << "while!" << std::endl;
                pX += s*delta;

                if( _tree_obj ) {
                    fcl::Vec3f translation(pX[0], pX[1], pX[2]);
                    fcl::Quaternion3f rotation(1, 0, 0, 0);                            
                    robotObject.setTransform(rotation, translation);
                    fcl::CollisionRequest requestType(1,false,1,false);
                    fcl::CollisionResult collisionResult;            
                    fcl::collide(&robotObject, &treeObj, requestType, collisionResult);
                 
                    obs = collisionResult.isCollision();
                }                
            }

            if( !obs ) {
                opt_poses.erase( opt_poses.begin()+i0+1, opt_poses.begin()+i1 );               
            }


        }

        /*
        auto end = boost::chrono::steady_clock::now();
        double ms  = boost::chrono::duration_cast<boost::chrono::milliseconds>(end - start).count();
        //std::cout << "MS: " << ms << std::endl;
        if ( ms > 1000 ) elapsed_time = true;
        */
        }
    } //Do it if you remove at lease one node

    return 1;

}




int PATH_PLANNER::plan(double max_t, std::vector<POSE> & poses, std::vector<POSE> & opt_poses) {

    //Planner not correctly initialized
    if( !_start_state_set || !_goal_state_set ) {
        std::cout << "Problem not correctly set: start or goal state not defined!" << std::endl;
        return -1;
    }

    if( !_rgeometry_set ) {
        std::cout << "Problem not correctly set: robot geometry not specified!" << std::endl;
        return -1;
    }

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


    //std::vector<POSE> opt_poses;

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


        optimize_path(poses, opt_poses);
        //opt_poses = poses;


        std::cout << "Pre-after optimization" << std::endl;

        std::cout << "Full path: " << poses.size() << std::endl;
        for ( int i=0; i<poses.size(); i++ ) {
            std::cout << "[" << i << "]: (" << poses[i].position.x << " " << poses[i].position.y << " " << poses[i].position.z << ")" << std::endl;
        }

        std::cout << "Optimized path: " << opt_poses.size() << std::endl;
        for ( int i=0; i<opt_poses.size(); i++ ) {
            std::cout << "[" << i << "]: (" << opt_poses[i].position.x << " " << opt_poses[i].position.y << " " << opt_poses[i].position.z << ")" << std::endl;
        }
    }
    reset_planner();

    //Return -1: wrong settings, 1: planned succeded, 0: no plan found
    return (solved) ? 1 : 0;
    
}