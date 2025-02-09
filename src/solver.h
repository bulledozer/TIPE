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
        float timeOfFlight(Traj*);
        float length(Traj*);
        std::pair<Traj,std::vector<float>> SolvePart(Road*, int, float,int,int, float, float);

};

#endif