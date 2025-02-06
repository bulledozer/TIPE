#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <iostream>

#include "src/traj.h"
#include "src/solver.h"
#include "src/spline.h"
#include "src/road.h"


void handleSpline(Spline*, Camera, int*);
void updateCam(Camera*, double, double);

int main(int, char**){
    const int screenWidth = 1280;
    const int screenHeight = 768;

    double theta = PI/2;
    double R = 30.0f;

    double camSpeed = 1.8f;
    double zoomSpeed = 50.0f;

    Camera3D camera = {0};
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    InitWindow(screenWidth, screenHeight, "TIPE");

    Traj traj = Traj(1.3f);
    //traj.CreateCircle(10.0f,50);
    
    Road road = Road(1.3f, 10);

    Solver solver;

    int selectedPoint = -1;
    Spline spline = Spline();


    while (!WindowShouldClose())
    {
        theta += (IsKeyDown(KEY_LEFT) - IsKeyDown(KEY_RIGHT))*GetFrameTime()*camSpeed;
        R += (IsKeyDown(KEY_DOWN) - IsKeyDown(KEY_UP))*GetFrameTime()*zoomSpeed;
        updateCam(&camera, theta, R);

        BeginDrawing();
        BeginMode3D(camera);
            ClearBackground(RAYWHITE);

            handleSpline(&spline, camera, &selectedPoint);
            //traj.CreateSpline(&spline, 500);
            road.CreateSpline(&spline, 100);
            Traj fastestPath = solver.Solve(&road, 10, 10,.20f);
            //Traj fastestPath = solver.SolvePart(&road, 10,.20f, 10,20);
            fastestPath.Draw(RED);

            std::vector<float> speed = solver.OptimalSpeed(&traj);
            std::vector<Color> col;
            for (auto &e : speed)
                {
                    Vector3 c = Vector3Lerp({ 135, 60, 190}, { 253, 249, 0}, 1.0f-1.0f/(1.0f+0.05f*e));
                    col.push_back(Color{(unsigned char)c.x,(unsigned char)c.y,(unsigned char)c.z,255});
                }

            DrawGrid(100,1.0f);
            //traj.Draw(col);
            road.Draw(RED);
        
        EndMode3D();
        DrawFPS(0,0);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}

void handleSpline(Spline * spline, Camera cam, int * selectedPoint)
{
    Vector2 mousePos = GetMousePosition();
    Ray camray = GetMouseRay(mousePos, cam);
    
    float t = -(camray.position.y)/(camray.direction.y);
    Vector3 inter = Vector3{camray.direction.x*t+camray.position.x, 0, camray.direction.z*t+camray.position.z};
    
    if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON))
    {
        spline->AddPoint(inter);
    }

    int focusedPoint = -1;
    for (int i = 0 ; i < spline->NumPoints() ; i++)
    {
        Vector3 point = spline->GetPoint(i);
        if (Vector3LengthSqr(Vector3Subtract(point, inter)) <= 1.0)
        {
            focusedPoint = i;
            if (IsMouseButtonDown(MOUSE_LEFT_BUTTON))
            {
                *selectedPoint = i;
                break;
            }
        }
    }

    spline->DrawControl(focusedPoint);

    if (*selectedPoint >= 0)
    {
        spline->ChangePoint(*selectedPoint, inter);
        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON))
            *selectedPoint = -1;
    }

}

void updateCam(Camera* cam, double theta, double R)
{
    cam->position = Vector3Scale(Vector3Normalize(Vector3{(float)(cos(theta)),1.0f,(float)(sin(theta))}), (float)R);
    cam->target = Vector3{0.0f,0.0f,0.0f};
}