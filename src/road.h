#include "raylib.h"

#include "spline.h"

#include <vector>

#ifndef ROAD_H
#define ROAD_H

class Road
{
    public:
        Road(float, int);

        void CreateSpline(Spline*,int);

        void Draw(Color);
        float mu;

        int NumPoints();
        int getRowSize();
        int getNumRows();

        Vector3 getNormalSide(int);
        Vector3 getInterpolated(int, float);
        std::vector<Vector3> points;
        Vector3 getPoint(int);
    private:
        void addRow(Vector3[]);
        int numRows = 0;
        const int rowSize;
};

#endif