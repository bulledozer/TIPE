#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <vector>
#include <iostream>

#include "traj.h"
#include "spline.h"


Traj::Traj(float mu)
{
    this->mu = mu;
}

int Traj::NumPoints() {return this->points.size();}

void Traj::CreateCircle(float radius,int res)
{
    std::vector<Vector3> t;

    for (int i = 0 ; i < res ; i++)
    {
        double arg = 2*PI*((float)i/(float)res);
        t.push_back(Vector3{radius*(float)cos(arg),0.0f, radius*(float)sin(arg)});
    }
    this->points = t;
}

void Traj::CreateSpline(Spline * spline, int res)
{
    std::vector<Vector3> t;
    for (int i = 0 ; i < res ; i++)
    {
        float m = (float)i/(float)res;
        t.push_back(spline->SampleAt(m));
    }
    this->points = t;
}

void Traj::addPoint(Vector3 point)
{
    this->points.push_back(point);
}

void Traj::changePoint(int pos, Vector3 point)
{
    this->points[pos] = point;
}

void Traj::append(Traj* tr)
{
    for (int i = 0 ; i < tr->NumPoints() ; i++)
    {
        this->addPoint(tr->getPoint(i));
    }
}

void Traj::Draw(std::vector<Color> col)
{
    for (int i = 1 ; i < col.size() ; i++)
    {
        //rlPushMatrix();
        //rlRotatef(90, 0,0,1);
        DrawCapsule(this->points[i], this->points[(i+1)%points.size()],0.3f,6,3, col[i]);
        //DrawCylinder(this->points[i], 0.5f, 0.5f, Vector3Length(Vector3Subtract(this->points[i], this->points[(i+1)%points.size()])), 6, col[i]);
        //rlPopMatrix();
    }
}

void Traj::Draw(Color col)
{
    for (int i = 1 ; i < this->points.size()-1 ; i++)
    {
        DrawCapsule(this->points[i], this->points[(i+1)%points.size()],0.3f,6,3, col);
    }
}

float Traj::GetRadius(int pos)
{
    Vector3 p1 = this->points[(pos)%this->NumPoints()];
    Vector3 p2 = this->points[(pos+1)%this->NumPoints()];

    Vector3 t1 = Vector3Subtract(this->points[(pos+1)%this->NumPoints()],this->points[(pos-1)%this->NumPoints()]);
    Vector3 t2 = Vector3Subtract(this->points[(pos+2)%this->NumPoints()],this->points[(pos)%this->NumPoints()]);

    float c1 = -(t1.x*p1.x+t1.z*p1.z);
    float c2 = -(t2.x*p2.x+t2.z*p2.z);

    float intery = (c1*(t2.x/t1.x)-c2)/(t2.z-t1.z*(t2.x/t1.x));
    Vector2 inter = Vector2{-(c1-t1.z*intery)/t1.x, intery};

    return Vector2Length(Vector2Subtract(Vector2{p1.x,p1.z}, inter));
}

Vector3 Traj::getPoint(int pos)
{
    return this->points[pos];
}