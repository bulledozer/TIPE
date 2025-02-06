#include "raylib.h"
#include "raymath.h"

#include <vector>

#ifndef SPLINE_H
#define SPLINE_H

class Spline
{

    public:
        Spline();

        int NumPoints();
        int NumSegment();

        void AddPoint(Vector3);
        Vector3 GetPoint(int);
        void ChangePoint(int, Vector3);

        Vector3 SampleAt(float);
        void DrawControl(int);

    private:
        std::vector<Vector3> points;
};

#endif