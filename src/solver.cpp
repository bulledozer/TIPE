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
    int res = 100;
    float vmax = 20.f;
    /*for (int i = 0 ; i < spline->NumPoints()-1 ; i++)
    {
        float d = Vector3Length(Vector3Subtract(spline->SampleAtCR((float)i/(float)spline->NumPoints()), 
                                                spline->SampleAtCR((float)(i+1)/(float)spline->NumPoints())));
        //std::cout << spline->SampleAtCR((float)i/(float)spline->NumPoints()).x << std::endl;
        t += d/(float)sqrt(spline->getRadiusAtCR((float)i/(float)spline->NumPoints())*G);

    }*/

    for (int i = 0; i < res ; i++)
    {
        float d = Vector3Length(Vector3Subtract(spline->SampleAtCR((float)i/(float)res), 
                                                spline->SampleAtCR((float)(i+1)/(float)res)));
        float v = (float)sqrt(spline->getRadiusAtCR((float)i/(float)res)*G*1.3f);
        t += d/(v < vmax ? v : vmax);
    }
    return t;
}

float Solver::ComputeTime(Road * road, std::vector<float> & state)
{
    std::vector<Vector3> controls = pointsFromState(road, state);

    float t = .0f;
    
    for (int i = 0 ; i < controls.size()-2 ; i++)
    {
        Vector3 A = Vector3Subtract(controls[i],controls[i+1]);
        Vector3 B = Vector3Subtract(controls[i+2],controls[i+1]);

        float L1 = Vector3Length(A);
        float L2 = Vector3Length(B); //caca

        float theta = acosf(Clamp(Vector3DotProduct(A,B)/(L1*L2),-1.f,1.f));
        
        t += 1/abs(tanf(theta/2));
    }

    /*for (int i = 0 ; i < controls.size()-1 ; i++)
    {
        float L = Vector3Length(Vector3Subtract(controls[i], controls[i+1]));

        t += L;
    }*/
    return t;
}

std::vector<Vector3> Solver::pointsFromState(Road * road, std::vector<float> & state)
{
    std::vector<Vector3> controls;

    for (int i = 0 ; i < state.size() ; i++)
    {
        float t = ((float)i/(float)state.size());
        controls.push_back(road->getInterpolated(((int)Lerp(.0f,(float)road->getNumRows(), t))*road->getRowSize(), state[i]));
    }
    //std::cout<< controls[5].x << std::endl;

    return controls;
}

Traj Solver::Solve(Road* road, int N, int iter, float strength)
{

    Traj sol = Traj(road->mu);

    std::vector<float> points;

    for (int i = 0 ; i < iter ; i++)
    {
        points.push_back(0.5f);
    } 
    
    for (int i = 0 ; i < N ; i++)
    {
        std::vector<float> gradient;
        float h = 0.00001f;
        
        float totaltime = this->ComputeTime(road, points);
        
        for (int j = 0 ; j < iter ; j++)
        {
            std::vector<float> new_state = points;
            //std::copy(points.begin(), points.end(), std::back_inserter(new_state));
            new_state[j] += h;
            new_state[j] = Clamp(new_state[j],.0f,1.f);

            float t =  ComputeTime(road, new_state);

            gradient.push_back((t-totaltime));
        }
        for (int j = 0 ; j < iter ; j++)
        {
            points[j] -= gradient[j]*strength;
            
            points[j] = Clamp(points[j],.0f,1.0f);
        }
    }
    std::vector<Vector3> sol_points = pointsFromState(road, points);
    //std::cout << sol_points.size() << std::endl;
    for (auto &e : sol_points)
    {
        //std::cout << spline.SampleAtCR((float)i/(float)road->getNumRows()).x << std::endl;
        //std::cout << e.x << std::endl;
        sol.addPoint(e);
    }
    //Vector3 c = Vector3Lerp({ 135, 60, 190}, { 253, 249, 0}, 1.0f-1.0f/(1.0f+0.05f*spline.getRadiusAtCR(.5f)));
    //DrawSphere(spline.SampleAtCR(.5f), 1.f, Color{(unsigned char)c.x,(unsigned char)c.y,(unsigned char)c.z,255});
    return sol;
}
