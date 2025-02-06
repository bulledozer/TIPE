#include <vector>
#include "raylib.h"

#include "spline.h"

#ifndef TRAJ_H
#define TRAJ_H

class Traj
{
    public:
        Traj(float);

        void CreateCircle(float,int);
        void CreateSpline(Spline*, int);
        void addPoint(Vector3);
        void append(Traj*);

        void changePoint(int, Vector3);

        void Draw(std::vector<Color>);
        void Draw(Color);
        float mu;

        int NumPoints();
        float GetRadius(int pos);
        Vector3 getPoint(int);

    private:
        std::vector<Vector3> points;
};

#endif