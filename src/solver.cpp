#include <iostream>
#include <vector>
#include <map>
#include "solver.h"
#include "spline.h"

#include "raymath.h"


bool Solver::IsValidVconst(Traj* traj, float speed)
{
    for (int i = 1 ; i < traj->NumPoints()-1 ; i++)
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

    for (int i = 1 ; i < traj->NumPoints()-1 ; i++)
    {  
        float R = traj->GetRadius2(i);
        float s2 = R*traj->mu*G;

        speed.push_back((float)sqrt(s2));
    }

    return speed;
}

float Solver::timeOfFlight(Traj* traj)
{
    std::vector<float> speed = this->OptimalSpeed(traj);
    float time = 0.0f;
    for (int i = 1 ; i < traj->NumPoints()-1 ; i++)
    {
        //time += (Vector3Length(Vector3Subtract(traj->getPoint(i), traj->getPoint(i+1)))/speed[i-1]);
        time += 1/speed[i-1];
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

float Solver::ComputeTime(Spline* spline)
{
    float t = 0.0f;
    for (int i = 0 ; i < spline->NumPoints()-1 ; i++)
    {
        float d = Vector3Length(Vector3Subtract(spline->SampleAtCR((float)i/(float)spline->NumPoints()), 
                                                spline->SampleAtCR((float)(i+1)/(float)spline->NumPoints())));
        //std::cout << spline->SampleAtCR((float)i/(float)spline->NumPoints()).x << std::endl;
        t += d/(float)sqrt(spline->getRadiusAtCR((float)i/(float)spline->NumPoints())*G);
    }
    return t;
}


Traj Solver::Solve(Road* road, int N, int iter, float strength)
{
    Traj sol = Traj(road->mu);

    Spline spline(road->getInterpolated(0,0.5f));
    std::vector<float> points;

    for (int i = 0 ; i < iter ; i++)
    {
        /*if (i == iter)
        {
            DrawSphere(road->getInterpolated(road->getRowSize()*(road->getNumRows()-1),.5f), 1.0f, GREEN);
            spline.AddSinglePoint(road->getInterpolated(road->getRowSize()*(road->getNumRows()-1),.5f));
            points.push_back(-1.f);
            continue;
        }*/
        float t = ((float)i/(float)iter);
        spline.AddSinglePoint(road->getInterpolated(((int)Lerp(.0f,(float)road->getNumRows(), t))*road->getRowSize(), 0.5f));
        points.push_back(0.5f);
    } 
    
    for (int i = 0 ; i < N ; i++)
    {
        std::vector<float> gradient;
        float h = 0.01f;
        
        float totaltime = this->ComputeTime(&spline);
        
        for (int j = 1 ; j < iter ; j++)
        {
            Vector3 p1 = road->getInterpolated((int)(((float)j/(float)iter)*road->getNumRows()*road->getRowSize()), points[j]+h);
            Vector3 tmp = spline.SampleAtCR((float)j/(float)spline.NumPoints());
            spline.ChangePoint(j, p1);
            float t = this->ComputeTime(&spline);
            
            spline.ChangePoint(j, tmp);
            
            gradient.push_back((t-totaltime));
        }
        for (int j = 1 ; j < iter ; j++)
        {
            points[j] -= gradient[j-1]*strength;
            
            points[j] = Clamp(points[j],.0f,1.0f);
            spline.ChangePoint(j, road->getInterpolated((int)(((float)j/(float)iter)*road->getNumRows()*road->getRowSize()), points[j]));
        }
    }
    for (int i = 0 ; i < road->getNumRows() ; i++)
    {
        //std::cout << spline.SampleAtCR((float)i/(float)road->getNumRows()).x << std::endl;
        sol.addPoint(spline.SampleAtCR((float)i/(float)road->getNumRows()));
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

/*std::pair<Traj,std::vector<float>> Solver::SolvePart(Road* road, int N, float strength,int im, int iM, float wm, float wM)
{
    std::vector<float> points;
    Spline spline(road->getInterpolated(0,0.5f));
    Traj traj(road->mu);

    float h = 0.001f;

    for (int i = 0 ; i < iM-im ; i++)
    {
        /*if (i==0) 
        {points.push_back(wm < 0 ? .5f : wm);
        traj.addPoint(road->getInterpolated((i+im)*road->getRowSize(), wm < 0 ? .5f : wm));}
        else if (i == iM-im-1)
        {points.push_back(wM < 0 ? .5f : wM);
        traj.addPoint(road->getInterpolated((i+im)*road->getRowSize(), wM < 0 ? .5f : wM));}
        else
        float t = Lerp(wm >= .0f ? wm : .5f,wM >= .0f ? wM : .5f, (float)i/(float)(iM-im));
        //std::cout << t << std::endl;
        points.push_back(t);
        traj.addPoint(road->getInterpolated((i+im)*road->getRowSize(), t));
        spline.AddPoint(road->getInterpolated((i+im)*road->getRowSize(), t));
    }
    
    for (int i = 0 ; i < N ; i++)
    {
        std::vector<float> gradient;

        float totallength = this->timeOfFlight(&traj);

        for (int j = im ; j < iM ; j++) 
        {
            Vector3 p1 = road->getInterpolated(j*road->getRowSize(), points[j-im]+h);
            Vector3 tmp = traj.getPoint(j-im);
            traj.changePoint(j-im, p1);

            float l = this->timeOfFlight(&traj);
            
            //traj.changePoint(j-im,tmp);

            gradient.push_back((l-totallength)/Vector3Length(Vector3Subtract(p1,tmp)));
        }
        for (int j = im ; j < iM ; j++)
        {
            if (j == im && wm >= 0)
                continue;
            if (j == iM-1 && wM >= 0)
                continue;
            points[j-im] -= gradient[j-im]*strength;
            
            points[j-im] = Clamp(points[j-im],.0f,1.0f);
            traj.changePoint(j-im, road->getInterpolated((j)*road->getRowSize(), points[j-im]));
        }
    }

    return std::pair<Traj,std::vector<float>>(traj,points);
}*/

/*Traj Solver::Solve(Road* road, int N, int iter, float strength)
{
    Traj sol = Traj(road->mu);
    std::vector<float> last;
    
    for (int i = 0 ; i < iter ; i++)
    {
        std::pair<Traj,std::vector<float>> seg = this->SolvePart(road, N, strength, (int)i*(road->getNumRows()/iter),(int)(i+1)*(road->getNumRows()/iter), last.size() == 0 ? -1.0f : last[last.size()-1],-1.0f);
        sol.append(&seg.first);
        last = seg.second;
    }
    return sol;
}*/