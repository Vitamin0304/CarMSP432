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
ParkTask::ParkTask()
{

}
void ParkTask::Init()
{
    step = 0;
    nowSpeed = parkSpeed;
    _car->MotorGoBack();
    pidDistanceSwitch = false;
}
int8_t ParkTask::Execute()
{
    bool nowError = false;
    //||(parkParam.flag == 1 && (parkInputs[0] <= 0 || parkInputs[1] <= 0 || inputs[2] <= -120 || inputs[2] >= 120))
    if(parkParam.flag == 0 && carStatus == park)
    {
        nowError = true;
    }
    lastError[0] = lastError[1];
    lastError[1] = lastError[2];
    lastError[2] = nowError;

    if(lastError[0]&&lastError[1]&&lastError[2])
    {
        Eigen::Vector3f x(parkInputs);
        carSim.SetVariables(x);

        carStatus = parkSim;
        return 0;
    }

    PARK_PARAM* param;
    if(carStatus == park && nowError == false)
    {
        param = &parkParam;
        parkInputs[0] = param->x + carLength*cos(param->theta /180*PI);
        parkInputs[1] = param->y + carLength*sin(param->theta /180*PI);
        parkInputs[2] = param->theta;
    }
    else if(carStatus == parkSim)
    {
        param = &parkSimParam;
        parkInputs[0] = param->x;
        parkInputs[1] = param->y;
        parkInputs[2] = param->theta;
    }


    if(parkInputs[0] <= 0 || parkInputs[1] <= 0
            || parkInputs[2] <= -120 || parkInputs[2] >= 120)
        return FINAL_STEP;  //进入错误状态

    switch(step)
    {
    case 0:
        //判断是否满足停止条件
        if(parkMethod == 0)
        {
            if(parkInputs[0]<=0.4 && parkInputs[0]>=0
                    && parkInputs[1]<=0.16 && parkInputs[1]>=0.06
                    && parkInputs[2]<=4 && parkInputs[2]>=-1.5)
            {
                ServoDirectionSet(0);
//                return FINAL_STEP;
                step++;
                break;
            }
            else
            {
                carSim.phi = fuzzy::parallel::CarFuzzy.Compute(parkInputs);
            }
        }else if(parkMethod == 1)
        {
            if(parkInputs[0]<=0.3 && parkInputs[0]>=-0.175
                    && parkInputs[1]<=1.2 && parkInputs[1]>=0.06
                    && parkInputs[2]<=110 && parkInputs[2]>=70)
            {
                carStatus = parkAdjust;
            }
            if(parkInputs[0]<=0.19 && parkInputs[0]>=0.06
                    && parkInputs[1]<=0.25 && parkInputs[1]>=0
                    && parkInputs[2]<=93 && parkInputs[2]>=87)
            {
                ServoDirectionSet(0);
//                return FINAL_STEP;
                step++;
                break;
            }
            else
            {
                carSim.phi = fuzzy::perpendicular::CarFuzzy.Compute(parkInputs);
            }
        }
        if(parkMethod == 0)
            ServoDirectionSet(carSim.phi/34);
        else
            ServoDirectionSet(-carSim.phi/33);
        break;
    case 1:
        if(parkMethod == 0)
        {
            _car->SetDistance(0.35-parkInputs[0]);
        }
        else if(parkMethod == 1)
        {
            _car->SetDistance(0.3-parkInputs[1]);
        }
        step++;
        break;
    case 2:
        float err = (_car->pidDistance[0].nowErr+_car->pidDistance[1].nowErr)/2;
        if(err<=0.01 && err>=-0.01)
        {
            step = -1;
        }
        break;
    default:
        break;
    }
    return step;
}

ParkAdjustTask::ParkAdjustTask()
{

}
void ParkAdjustTask::Init()
{
    step = 0;
    pidParkAdjust->firstCompute = true;
    nowSpeed = parkSpeed;
    _car->MotorGoBack();
    pidDistanceSwitch = false;
}
int8_t ParkAdjustTask::Execute()
{
    parkInputs[0] = parkSimParam.x;
    parkInputs[1] = parkSimParam.y;
    parkInputs[2] = parkSimParam.theta;

    switch(step)
    {
    case 0:
        if(parkMethod == 1)
        {
            if(parkInputs[0]<=0.19 && parkInputs[0]>=0.06
                    && parkInputs[1]<=0.3 && parkInputs[1]>=0
                    && parkInputs[2]<=92 && parkInputs[2]>=88)
            {
                ServoDirectionSet(0);
                step++;
                break;
            }
            else
            {
                carSim.phi = pidParkAdjust->Compute()+0.8*(parkInputs[2]-90);
                if(carSim.phi>25)
                    carSim.phi=25;
                else if(carSim.phi<-25)
                    carSim.phi=-25;
            }
        }
        ServoDirectionSet(-carSim.phi/33);
        break;
    case 1:
        if(parkMethod == 1)
        {
            _car->SetDistance(0.3-parkInputs[1]);
        }
        step++;
        break;
    case 2:
        float err = (_car->pidDistance[0].nowErr+_car->pidDistance[1].nowErr)/2;
        if(err<=0.01 && err>=-0.01)
        {
            step = -1;
        }
        break;
    default:
        break;
    }
    return 0;
}

PathTrackTask::PathTrackTask()
{
}
void PathTrackTask::Init()
{
    step = 0;
    nowSpeed = parkSpeed;
    _car->MotorGoBack();
    pidDistanceSwitch = false;
}


}
