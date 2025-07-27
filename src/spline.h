#include "raylib.h"
#include "raymath.h"

#include <vector>

#ifndef SPLINE_H
#define SPLINE_H

class Spline
{

    public:
        Spline(bool);
        Spline(Vector3);

        int NumPoints();
        int NumSegment();

        void AddPoint(Vector3);
        void AddSinglePoint(Vector3);
        Vector3 GetPoint(int);
        void ChangePoint(int, Vector3);

        Vector3 SampleAt(float);
        Vector3 SampleAtCR(float);
        float getRadiusAtCR(float);
        void DrawControl(int);

    private:
        std::vector<Vector3> points;

};

#endif