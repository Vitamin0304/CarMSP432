/*
 * Task.cpp
 *
 *  Created on: 2021年3月31日
 *      Author: 电脑
 */

#include <Apps/Task.h>
#include <Apps/Car.h>
#include <Apps/motor_timer.h>
#include <cmath>
#include <Eigen/Core>
#include <list>

extern "C" {
#include "Apps/my_uart.h"
}

namespace carTask
{
TaskBase::TaskBase()
{
    // TODO Auto-generated constructor stub

}

TaskBase::~TaskBase()
{
    // TODO Auto-generated destructor stub
}

PathTrackSimTask::PathTrackSimTask()
{

}

void PathTrackSimTask::Init()
{
    step = 0;
    nowSpeed = parkSpeed;
//    _car->MotorGoBack();
    pidDistanceSwitch = false;
}
int8_t PathTrackSimTask::Execute2(float v_r)
{
    static bool stop;
    Vector2f z;
    float e;
    static float& phi = pathTrackSim.carSim->phi;

    static Vector3f& x = pathTrackSim.carSim->x;

    switch(step)
    {
    case 0:
    {
        if(parkParamIsNew)
        {
            x[0] = parkParam.x;
            x[1] = parkParam.y;
            x[2] = parkParam.theta;
            parkParamIsNew = false;
        }
        else
        {
            pathTrackSim.carSim->ComputeSim_IMU(v_r, parkMethod);
        }
        stop = pathTrackSim.ComuputeError(z, e);
        if (stop)
        {
            ServoDirectionSet(0);
            return -1;
        }
        uint16_t nowPath = 0;
        vector<uint16_t>& unsmooth = pathTrackSim.pathGenerator->unsmoothSteps;
        for (int j = 0; j < unsmooth.size(); ++j)
        {
            //处理小车行驶时改变方向问题
            if (pathTrackSim.nowStep >= unsmooth[j] - 3
                && pathTrackSim.nowStep < unsmooth[j]
                && parkMethod == 0)
            {
                pathTrackSim.nowStep = unsmooth[j];
                stop = pathTrackSim.ComuputeError(z, e);
                if (stop)
                {
                    ServoDirectionSet(0);
                    return -1;
                }
                nowPath = j+1;
                break;
            }
            if (pathTrackSim.nowStep == unsmooth[j])
            {
                pidPathTrack.integral = 0;
            }
            if (pathTrackSim.nowStep < unsmooth[j])
            {
                nowPath = j;
                break;
            }
        }

        bool nowDirection = pathTrackSim.pathGenerator->directions[nowPath];
        if (nowDirection == true)
            _car->MotorRun();
        else
            _car->MotorGoBack();
        phi = pathTrackSim.Stanley(z, x[2], e, v_r, nowDirection);

        phi = pidPathTrack.Compute();

//        if(parkMethod == 1)
//            phi *= 28.0/15.0;

        float phi_out = phi *180/EIGEN_PI;

        if(parkMethod == 0)
            ServoDirectionSet(phi_out/28);
        else
            ServoDirectionSet(phi_out/28);

        break;
    }
    default:
        break;
    }
    return step;
}

}
