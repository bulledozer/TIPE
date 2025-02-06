#include "spline.h"

#include <iostream>

Spline::Spline()
{
    points.push_back(Vector3{10.0f,.0f,-10.0f});
    points.push_back(Vector3{10.0f,.0f,10.0f});
    points.push_back(Vector3{-10.0f,.0f,10.0f});
    points.push_back(Vector3{-10.0f,.0f,-10.0f});
}

int Spline::NumPoints() {return this->points.size();}

int Spline::NumSegment()
{
    return (this->NumPoints()-4)/3+1;
}

void Spline::AddPoint(Vector3 point)
{
    this->points.push_back(Vector3Add(this->points[this->points.size()-1], Vector3{10.0f,.0f,.0f}));
    this->points.push_back(Vector3Add(point, Vector3{10.0f,.0f,.0f}));
    this->points.push_back(point);
}

Vector3 Spline::GetPoint(int n) {return this->points[n%this->NumPoints()];}

void Spline::ChangePoint(int n, Vector3 dir)
{
    this->points[n] = dir;
}

Vector3 Spline::SampleAt(float t)
{
    int segment = (int)floor(t*(this->NumSegment()));
    
    Vector3 A = this->points[(segment*3)%this->NumPoints()];
    Vector3 B = this->points[(segment*3+1)%this->NumPoints()];
    Vector3 C = this->points[(segment*3+2)%this->NumPoints()];
    Vector3 D = this->points[(segment*3+3)%this->NumPoints()];

    float nt = Wrap(t,0.0f, 1.0f/(float)this->NumSegment())*this->NumSegment();

    Vector3 p0 = Vector3Lerp(Vector3Lerp(A,B,nt),Vector3Lerp(B,C,nt),nt);
    Vector3 p1 = Vector3Lerp(Vector3Lerp(B,C,nt),Vector3Lerp(C,D,nt),nt);

    return Vector3Lerp(p0,p1,nt);
}

void Spline::DrawControl(int focus)
{
    for (int i = 0 ; i < this->NumPoints() ; i++)
    {
        DrawSphere(this->points[i], .5f, (i==focus)?BLUE:RED);
    }
}