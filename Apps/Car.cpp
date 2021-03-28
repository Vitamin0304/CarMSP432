/*
 * Car.cpp
 *
 *  Created on: 2021年2月8日
 *      Author: 电脑
 */
extern "C" {
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
}
#include <Apps/Motor.h>
#include <Apps/PIDController.h>
#include <Apps/Car.h>
#include <Apps/Encoder.h>
#include <Apps/motor_timer.h>
#include <cmath>
#include <Eigen/Core>

volatile enum CarStatus carStatus = stop;
volatile enum CarStatus previousStatus = stop;

float defaultSpeed[2] = {5,5};
float parkSpeed[2] = {2,2};

fuzzy::CarSim carSim(0.142, delay_s);

Car::Car(Motor* motors,
         PIDSpeedController* pidSpeed,
         PIDDistanceController* pidDistance)
{
    this->motors = motors;
    this->pidSpeed = pidSpeed;
    this->pidDistance = pidDistance;
    ServoInit();
    ServoDirectionSet(0);
}

Car::~Car()
{
    // TODO Auto-generated destructor stub
}
void Car::MotorRun()
{
    motors[0].omega_set = nowSpeed[0];
    motors[1].omega_set = nowSpeed[1];
}
void Car::MotorStop()
{
    motors[0].omega_set = 0;
    motors[1].omega_set = 0;
}
void Car::MotorGoBack()
{
    motors[0].omega_set = -nowSpeed[0];
    motors[1].omega_set = -nowSpeed[1];
}
void Car::SetDistance(float distance)
{
    pidDistanceSwitch = true;
    pidDistance[0].outputMax = nowSpeed[0];
    pidDistance[1].outputMax = nowSpeed[1];
    pidDistance[0].Reset();
    pidDistance[1].Reset();
    motors[0].distance_actual = 0;
    motors[1].distance_actual = 0;
    motors[0].distance_set = distance;
    motors[1].distance_set = distance;
}

void Car::StatusManage()
{
    if(carStatus == previousStatus)    //指令没改变
    {
        return;
    }
    switch(carStatus)
    {
        case run:    //1号键  启动
           nowSpeed = defaultSpeed;
           MotorRun();
           pidDistanceSwitch = false;
           break;
       case stop:    //2号键  停止
           MotorStop();
           pidDistanceSwitch = false;
           break;
       case brake:   //4号键 刹车
           SetDistance(0);
           break;
       case commandGoForward:    //前进键
           nowSpeed = defaultSpeed;
           MotorRun();
           pidDistanceSwitch = false;
           break;
       case commandGoBack:    //后退键
           nowSpeed = defaultSpeed;
           MotorGoBack();
           pidDistanceSwitch = false;
           break;
       case commandTurnLeft:    //左键
           ServoDirectionSet(-1);
           break;
       case commandTurnRight:    //右键
           ServoDirectionSet(1);
           break;
       case commandGoStraight:   //舵机直行
           ServoDirectionSet(0);
           break;
       case park:
           nowSpeed = parkSpeed;
           MotorGoBack();
           break;
       case parkSim:
           if(previousStatus != park){
               Eigen::Vector3f x(parkInit.Kp, parkInit.Ki, parkInit.Kd);
               carSim.SetVariables(x);
               parkMethod = (uint8_t)parkMethodFloat;
           }
           nowSpeed = parkSpeed;
           MotorGoBack();
           break;
       case imageSeekLine:

           break;
       case obstacle:            //红外检测到障碍物
           break;
    }
    pidSpeed[0].integral = 0;
    pidSpeed[1].integral = 0;
    previousStatus = carStatus;
}

void Car::PIDCompute()
{
    motors[0].ComputeActualParam();
    motors[1].ComputeActualParam();

    if(pidDistanceSwitch)
    {
        omega_set_output[0] = pidDistance[0].Compute();
        omega_set_output[1] = pidDistance[1].Compute();
        motors[0].omega_set = omega_set_output[0];
        motors[1].omega_set = omega_set_output[1];
    }

    motor_output[0] = pidSpeed[0].Compute();
    motor_output[1] = pidSpeed[1].Compute();
    motors[0].SetSpeed(motor_output[0]);
    motors[1].SetSpeed(-motor_output[1]);
}

void Car::ExecuteTask()
{
    switch(carStatus)
    {
    case park:
        if(ParkTask() == -1)
            carStatus = stop;
        break;
    case parkSim:
        carSim.Compute(-(motors[0].omega_actual + motors[1].omega_actual)/2*0.0325, 1);
        if(ParkTask() == -1)
            carStatus = stop;
        break;
    default:
        break;
    }
}
PARK_PARAM Car::parkParam = {0,0,0};
PARK_PARAM Car::parkSimParam = {0,0,0};
uint8_t Car::parkMethod = 1;
PID_PARAM Car::parkInit = {0,0,0};
float Car::parkMethodFloat = 0;

bool Car::lastError[3] = {false};
int8_t Car::ParkTask()
{
    static int32_t step = 0;
    static float inputs[3] = {0};

    bool nowError = false;
    //||(parkParam.flag == 1 && (inputs[0] <= 0 || inputs[1] <= 0 || inputs[2] <= -120 || inputs[2] >= 120))
    if(parkParam.flag == 0 && carStatus == park)
    {
        nowError = true;
    }
    lastError[0] = lastError[1];
    lastError[1] = lastError[2];
    lastError[2] = nowError;

    if(lastError[0]&&lastError[1]&&lastError[2])
    {
        Eigen::Vector3f x(inputs);
        carSim.SetVariables(x);

        carStatus = parkSim;
        return 0;
    }

    PARK_PARAM* param;
    if(carStatus == park && nowError == false)
    {
        param = &parkParam;
        inputs[0] = param->x + carLength*cos(param->theta /180*PI);
        inputs[1] = param->y + carLength*sin(param->theta /180*PI);
        inputs[2] = param->theta;
    }
    else if(carStatus == parkSim)
    {
        param = &parkSimParam;
        inputs[0] = param->x;
        inputs[1] = param->y;
        inputs[2] = param->theta;
    }


    if(inputs[0] <= 0 || inputs[1] <= 0
            || inputs[2] <= -120 || inputs[2] >= 120)
        return FINAL_STEP;  //进入错误状态

    switch(step)
    {
    case 0:
        //判断是否满足停止条件
        if(parkMethod == 0)
        {
            if(inputs[0]<=0.3 && inputs[0]>=0
                    && inputs[1]<=0.16 && inputs[1]>=0.06
                    && inputs[2]<=1 && inputs[2]>=-1)
            {
                ServoDirectionSet(0);
                return FINAL_STEP;
            }
            else
            {
                carSim.phi = fuzzy::parallel::CarFuzzy.Compute(inputs);
            }
        }else if(parkMethod == 1)
        {
            if(inputs[0]<=0.19 && inputs[0]>=0.06
                    && inputs[1]<=0.25 && inputs[1]>=0
                    && inputs[2]<=91 && inputs[2]>=89)
            {
                ServoDirectionSet(0);
                return FINAL_STEP;
            }
            else
            {
                carSim.phi = fuzzy::perpendicular::CarFuzzy.Compute(inputs);
            }
        }
        if(parkMethod == 0)
            ServoDirectionSet(carSim.phi/25);
        else
            ServoDirectionSet(-carSim.phi/25);
        break;
    default:
        break;
    }
    return step;
}


void Car::CommandQuery(uint32_t infraredCommand)
{
    switch(infraredCommand)
    {
    case 0x00FFA25D:    //1号键  启动
        carStatus = run;
        break;
    case 0x00FF629D:    //2号键  停止
        carStatus = stop;
        break;
    case 0x00FF22DD:    //4号键 刹车
        carStatus = brake;
        break;
    case 0x00FF18E7:    //前进键
        carStatus = commandGoForward;
        break;
    case 0x00FF4AB5:    //后退键
        carStatus = commandGoBack;
        break;
    case 0x00FF10EF:    //左键
        carStatus = commandTurnLeft;
        break;
    case 0x00FF5AA5:    //右键
        carStatus = commandTurnRight;
        break;
    case 0x00FF38C7:    //舵机直行
        carStatus = commandGoStraight;
        break;
    case 0x00FFE21D:    //3键  寻线
        carStatus = parkSim;
        parkMethod = 1;
        break;
    case 0x00FF02FD:    //5键  平行泊车
        carStatus = park;
        parkMethod = 0;
        break;
    case 0x00FFC23D:    //6键  垂直泊车
        carStatus = park;
        parkMethod = 1;
        break;
    }
}
