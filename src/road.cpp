#include "road.h"

#include "raymath.h"

#include <vector>
#include <iostream>

Road::Road(float mu, int rs) : rowSize(rs)
{
    this->mu = mu;
}

void Road::addRow(Vector3 row[])
{
    for (int i = 0 ; i < this->rowSize ; i++)
    {
        this->points.push_back(row[i]);
    }
    numRows++;
}

int Road::NumPoints() {return this->points.size();}
int Road::getRowSize() {return this->rowSize;}
int Road::getNumRows() {return this->numRows;}

void Road::CreateSpline(Spline* spline, int res)
{
    float width = 3.0f;
    this->points.clear();
    numRows = 0;
    for (int i = 0 ; i < res ; i++)
    {
        float m = (float)i/(float)res;
        
        Vector3 p1 = spline->SampleAt(m);
        Vector3 p2 = spline->SampleAt(m+(1/(float)res));

        Vector3 normalSide = Vector3CrossProduct(Vector3Subtract(p2,p1),Vector3{0.0f,1.0f,0.0f});
        normalSide = Vector3Normalize(normalSide);

        Vector3 (*row) = new Vector3[this->rowSize];
        for (int j = 0 ; j < this->rowSize ; j++)
        {
            float n = ((float)j/(float)this->rowSize)*2.0f-1.0f;
            row[j] = Vector3Add(p1,Vector3Scale(normalSide, width*n));
        }
        this->addRow(row);
        delete[] row;
    }
}

Vector3 Road::getNormalSide(int pos)
{
    Vector3 p1 = this->points[pos];
    Vector3 p2 = this->points[pos+1];
    Vector3 normalSide = Vector3CrossProduct(Vector3Subtract(p2,p1),Vector3{0.0f,1.0f,0.0f});
    normalSide = Vector3Normalize(normalSide);

    return normalSide;
}

Vector3 Road::getInterpolated(int pos, float m)
{
    return Vector3Lerp(this->points[pos], this->points[pos+this->rowSize-1],m);
}

Vector3 Road::getPoint(int pos) {return this->points[pos];}

void Road::Draw(Color col)
{
    for (int i = 0 ; i < this->numRows-2 ; i++)
    {
        //DrawTriangle3D(this->points[i*this->rowSize],this->points[(i+1)*this->rowSize-1],this->points[(i+1)*this->rowSize],col);
        //DrawTriangle3D(this->points[(i+1)*this->rowSize-1],this->points[(i+2)*this->rowSize-1],this->points[(i+1)*this->rowSize], col);
        DrawLine3D(this->points[i*this->rowSize], this->points[(i+1)*this->rowSize-1], col);
        DrawLine3D(this->points[i*this->rowSize], this->points[(i+1)*this->rowSize], col);
        DrawLine3D(this->points[(i+1)*this->rowSize], this->points[(i+2)*this->rowSize-1], col);
        DrawLine3D(this->points[(i+2)*this->rowSize-1], this->points[(i+1)*this->rowSize-1], col);
    }
}