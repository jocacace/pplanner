#include "planner.h"

using namespace std;

int main(int argc, char** argv) {

    double xbounds[2];
    double ybounds[2];
    double zbounds[2];

    xbounds[0] = -20.0;
    xbounds[1] =  20.0;
    ybounds[0] = -20.0;
    ybounds[1] =  20.0;
    zbounds[0] =  0.0;
    zbounds[1] =  10.0;

    PATH_PLANNER pp(xbounds, ybounds, zbounds);
       
    pp.set_random_start_state();
    pp.set_random_goal_state();

    std::vector<POSE> poses;
    int ret = pp.plan(1.0, poses);
    if( ret < 0 ) 
        cout << "Planner not correctly initialized" << endl;
    else {

        cout << "Solution: " << endl;
        for(int i=0; i<poses.size(); i++ ) {
            cout << "Pose: [" << i << "]: " << "(" << poses[i].position.x << " " << poses[i].position.y << " " << poses[i].position.z << "), " <<
            "(" << poses[i].orientation.w << " " << poses[i].orientation.x << " " << poses[i].orientation.y << " " << poses[i].orientation.z << ")" << endl;
        }
    }
    return -1;
}