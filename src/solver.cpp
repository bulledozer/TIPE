#include <iostream>
#include <vector>
#include <map>
#include "solver.h"

#include "raymath.h"


bool Solver::IsValidVconst(Traj* traj, float speed)
{
    for (int i = 1 ; i < traj->NumPoints() ; i++)
    {
        float R = traj->GetRadius(i);
        float s2 = R*traj->mu*G;

        if (s2 < speed*speed)
            return 0;
    }
    return 1;
}

std::vector<float> Solver::OptimalSpeed(Traj* traj)
{
    std::vector<float> speed;

    for (int i = 1 ; i < traj->NumPoints() ; i++)
    {
        float R = traj->GetRadius(i);
        float s2 = R*traj->mu*G;

        speed.push_back((float)sqrt(s2));
    }

    return speed;
}

std::vector<float> Solver::timeOfFlight(Traj* traj)
{
    std::vector<float> speed = this->OptimalSpeed(traj);
    std::vector<float> time;
    for (int i = 1 ; i < traj->NumPoints()-1 ; i++)
    {
        time.push_back(Vector3Length(Vector3Subtract(traj->getPoint(i), traj->getPoint(i+1)))/speed[i]);
    }
    return time;
}

float Solver::length(Traj* traj)
{
    float l = .0f;
    for (int i = 0 ; i < traj->NumPoints()-1 ; i++)
    {
        l += Vector3Length(Vector3Subtract(traj->getPoint(i), traj->getPoint(i+1)));
    }
    return l;
}

Traj Solver::SolvePart(Road* road, int N, float strength,int im, int iM)
{
    std::vector<float> points;
    Traj traj(road->mu);

    float h = 0.001f;

    for (int i = 0 ; i < iM-im ; i++)
    {
        points.push_back(0.5f);
        traj.addPoint(road->getInterpolated((i+im)*road->getRowSize(), 0.5f));
    }
    
    for (int i = 0 ; i < N ; i++)
    {
        std::vector<float> gradient;

        float totallength = this->length(&traj);

        for (int j = im ; j < iM ; j++) 
        {
            Vector3 p1 = road->getInterpolated(j*road->getRowSize(), points[j-im]+h);
            Vector3 tmp = traj.getPoint(j-im);
            traj.changePoint(j-im, p1);

            float l = this->length(&traj);
            
            gradient.push_back((l-totallength)/Vector3Length(Vector3Subtract(p1,tmp)));
        }
        for (int j = im ; j < iM ; j++)
        {
            points[j-im] -= gradient[j-im]*strength;
            
            points[j-im] = Clamp(points[j-im],.0f,1.0f);
            traj.changePoint(j-im, road->getInterpolated((j)*road->getRowSize(), points[j-im]));
        }
    }

    return traj;
}

Traj Solver::Solve(Road* road, int N, int iter, float strength)
{
    Traj sol = Traj(road->mu);

    for (int i = 0 ; i < iter ; i++)
    {
        Traj seg = this->SolvePart(road, N, strength, (int)i*(road->getNumRows()/iter),(int)(i+1)*(road->getNumRows()/iter));
        sol.append(&seg);
    }
    return sol;
}









//---------------------- ZONE CACA ------------------------------


/*
        std::vector<float> time = this->timeOfFlight(&traj);
        float totaltime = 0.0f;
        for (int k = 0 ; k < road->getNumRows()-2 ; k++)
            {
                totaltime += time[k];
            }*/
        //std::cout << totaltime << std::endl;

            //traj.changePoint(j-im,tmp);

            /*float d1 = Vector3Length(Vector3Subtract(p1, road->getInterpolated((j-1)*road->getRowSize(), points[j-1])));
            float d2 = Vector3Length(Vector3Subtract(p1, road->getInterpolated((j+1)*road->getRowSize(), points[j+1])));
            float R = traj.GetRadius(j);
            float v = (float)sqrt(road->mu*G*R);//pipicaca
            //std::cout << v << std::endl;
            //td::cout << Vector3Length(Vector3Subtract(p1,tmp)) << std::endl;
            //gradient.push_back((totaltime-time[j-2]-time[j]+(d1+d2)/v)/Vector3Length(Vector3Subtract(p1,tmp)));
            //float t = (totaltime-time[j-2]-time[j-1]+(d1+d2)/v)/Vector3Length(Vector3Subtract(p1,tmp));
            gradient.push_back((-time[j-2]-time[j-1]+(d1+d2)/v)/Vector3Length(Vector3Subtract(p1,tmp)));*/
            //if (t < 0.0f)
            //std::cout << -time[j-2]-time[j-1]+(d1+d2)/v  << std::endl;