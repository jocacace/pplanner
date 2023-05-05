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

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>


#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/ccd/motion.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"

#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>



namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef struct POSITION {
    double x;
    double y;
    double z;
}POSITION;

typedef struct ORIENTATION {
    double w;
    double x;
    double y;
    double z;
}ORIENTATION;


class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }

    // Our requirement is to maximize path clearance from obstacles,
    // but we want to represent the objective as a path cost
    // minimization. Therefore, we set each state's cost to be the
    // reciprocal of its clearance, so that as state clearance
    // increases, the state cost decreases.
    ob::Cost stateCost(const ob::State* s) const override
    {
        return ob::Cost(1 / (si_->getStateValidityChecker()->clearance(s) +
            std::numeric_limits<double>::min()));
    }
};

typedef struct POSE {
    POSITION position;
    ORIENTATION orientation;
}POSE;

class PATH_PLANNER {
    public:
        PATH_PLANNER(double xbounds[2], double ybounds[2], double zbounds[2]);
        int plan(double max_t, std::vector<POSE> & poses);
        bool isStateValid(const ob::State *state);

        void reset_planner() {
            if( !_random_start_state ) _start_state_set = false;
            if( !_random_goal_state ) _goal_state_set = false;
        }
        
        void set_start_state( POSE s) {
            _start = s;
            _random_start_state = false;
            _start_state_set = true;
        }

        void set_goal_state( POSE g) {
            _goal = g;
            _random_goal_state = false;
            _goal_state_set = true;
        }
    
        void set_verbose( int v ) {
            if ( v == 0 ) 
                _verbose = false;
            if ( v == 1 )  
                _verbose = true;
        }

        void set_random_start_state () {
            _random_start_state = true;
            _start_state_set = true;
        }

        void set_random_goal_state () {
            _random_goal_state = true;
            _goal_state_set = true;
        }
        void set_octo_tree(octomap::OcTree* t ) {
            fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(t));
            _tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);
        }

    private:
        bool _start_state_set = false;
        bool _goal_state_set = false;
        bool _random_start_state = false;
        bool _random_goal_state = false;
		bool _verbose = false;

        ob::StateSpacePtr _space;
  		ob::SpaceInformationPtr _si;
        ob::ProblemDefinitionPtr _pdef;
        ob::PlannerPtr _planner;
        
        POSE _start;
        POSE _goal;

        fcl::OcTree* _tree;
        std::shared_ptr<fcl::CollisionGeometry> _tree_obj;
        std::shared_ptr<fcl::CollisionGeometry> _Robot;
};
