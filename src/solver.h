#include <vector>

#include "traj.h"
#include "road.h"

#include "raymath.h"

#ifndef SOLVER_H
#define SOLVER_H

class Solver
{
    const float G = 9.81f;

    public:
        bool IsValidVconst(Traj*,float);
        std::vector<float> OptimalSpeed(Traj*);
        Traj Solve(Road*,int,int,float);
    private:
        std::vector<float> timeOfFlight(Traj*);
        float length(Traj*);
        Traj SolvePart(Road*, int, float,int,int);

};

#endif