#include "spline.h"

#include <iostream>

Spline::Spline()
{
    points.push_back(Vector3{10.0f,.0f,-10.0f});
    points.push_back(Vector3{10.0f,.0f,10.0f});
    points.push_back(Vector3{-10.0f,.0f,10.0f});
    points.push_back(Vector3{-10.0f,.0f,-10.0f});
}

Spline::Spline(Vector3 p)
{
    points.push_back(p);
}

int Spline::NumPoints() {return this->points.size();}

int Spline::NumSegment()
{
    return (this->NumPoints()-4)/3+1;
}

void Spline::AddSinglePoint(Vector3 point)
{
    this->points.push_back(point);
}

void Spline::AddPoint(Vector3 point)
{
    this->points.push_back(Vector3Lerp(this->points[this->points.size()-1], point, 0.333f));
    this->points.push_back(Vector3Lerp(this->points[this->points.size()-1], point, 0.666f));
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

Vector3 Spline::SampleAtCR(float t)
{
    int segment = (int)floor(t*(this->NumPoints()-3));

    Vector3 A = this->points[(segment)%this->NumPoints()];
    Vector3 B = this->points[(segment+1)%this->NumPoints()];
    Vector3 C = this->points[(segment+2)%this->NumPoints()];
    Vector3 D = this->points[(segment+3)%this->NumPoints()];

    float nt = Wrap(t,0.0f, 1.0f/(float)(this->NumPoints()-3))*(float)(this->NumPoints()-3);
    //float nt = t;

    Vector3 t1 = Vector3Scale(B,2.0f);
    Vector3 t2 = Vector3Scale(Vector3Subtract(C,A), nt);
    Vector3 t3 = Vector3Scale(Vector3Subtract(Vector3Add(Vector3Subtract(Vector3Scale(A,2.0f), Vector3Scale(B,5.0)), Vector3Scale(C,4.0)),D), nt*nt);
    Vector3 t4 = Vector3Scale(Vector3Add(Vector3Subtract(Vector3Subtract(Vector3Scale(B,3.0f), A), Vector3Scale(C, 3.0f)), D), nt*nt*nt);

    return Vector3Scale(Vector3Add(Vector3Add(Vector3Add(t1,t2),t3),t4),0.5f);
}

float Spline::getRadiusAtCR(float t)
{
    int segment = (int)floor(t*(this->NumPoints()-3));

    Vector3 A = this->points[(segment)%this->NumPoints()];
    Vector3 B = this->points[(segment+1)%this->NumPoints()];
    Vector3 C = this->points[(segment+2)%this->NumPoints()];
    Vector3 D = this->points[(segment+3)%this->NumPoints()];

    float dxdt = 0.5f * (C.x-A.x + 2.0f*t*(2.0f*A.x-5.0f*B.x+4.0f*C.x-D.x)+3.f*t*t*(-A.x+3.f*B.x-3.f*C.x+D.x));
    float dzdt = 0.5f * (C.z-A.z + 2.0f*t*(2.0f*A.z-5.0f*B.z+4.0f*C.z-D.z)+3.f*t*t*(-A.z+3.f*B.z-3.f*C.z+D.z));
    float dx2dt = 0.5f * (2.0f*(2.0f*A.x-5.0f*B.x+4.0f*C.x-D.x)+6.f*t*(-A.x+3.f*B.x-3.f*C.x+D.x));
    float dz2dt = 0.5f * (2.0f*(2.0f*A.z-5.0f*B.z+4.0f*C.z-D.z)+6.f*t*(-A.z+3.f*B.z-3.f*C.z+D.z));

    return abs(pow(dxdt*dxdt+dzdt*dzdt, 1.5f)/(dxdt*dz2dt-dzdt*dx2dt));
}

void Spline::DrawControl(int focus)
{
    for (int i = 0 ; i < this->NumPoints() ; i++)
    {
        DrawSphere(this->points[i], .5f, (i==focus)?BLUE:RED);
    }
}