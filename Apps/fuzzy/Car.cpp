/*
 * Car.cpp
 *
 *  Created on: 2021��2��8��
 *      Author: ����
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
#include <Apps/Task.h>
namespace carTask
{
volatile enum CarStatus carStatus = stop;
volatile enum CarStatus previousStatus = stop;

float defaultSpeed[2] = {1.5,1.5};
float parkSpeed[2] = {1.5,1.5};

float* nowSpeed = defaultSpeed;
bool pidDistanceSwitch = false;

fuzzy::CarSim carSim(0.142, delay_s);
PIDParkAdjustController* pidParkAdjust = NULL;
Car* _car = NULL;

ParkTask parkTask;
ParkAdjustTask parkAdjustTask;

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
    if(carStatus == previousStatus)    //ָ��û�ı�
    {
        return;
    }
    switch(carStatus)
    {
        case run:    //1�ż�  ����
           nowSpeed = defaultSpeed;
           MotorRun();
           pidDistanceSwitch = false;
           break;
       case stop:    //2�ż�  ֹͣ
           MotorStop();
           pidDistanceSwitch = false;
           break;
       case brake:   //4�ż� ɲ��
           SetDistance(0);
           break;
       case commandGoForward:    //ǰ����
           nowSpeed = defaultSpeed;
           MotorRun();
           pidDistanceSwitch = false;
           break;
       case commandGoBack:    //���˼�
           nowSpeed = defaultSpeed;
           MotorGoBack();
           pidDistanceSwitch = false;
           break;
       case commandTurnLeft:    //���
           ServoDirectionSet(-1);
           break;
       case commandTurnRight:    //�Ҽ�
           ServoDirectionSet(1);
           break;
       case commandGoStraight:   //���ֱ��
           ServoDirectionSet(0);
           break;
       case park:
//           nowSpeed = parkSpeed;
//           MotorGoBack();
//           pidDistanceSwitch = false;
           parkTask.Init();
           break;
       case parkSim:
           if(previousStatus != park){
               Eigen::Vector3f x(parkInit.Kp, parkInit.Ki, parkInit.Kd);
               carSim.SetVariables(x);
               parkMethod = (uint8_t)parkMethodFloat;
           }
//           nowSpeed = parkSpeed;
//           MotorGoBack();
//           pidDistanceSwitch = false;
           parkTask.Init();
           break;
       case parkAdjust:
           if(previousStatus != park && previousStatus != parkSim){
               Eigen::Vector3f x(0.3,0.9,70);
               carSim.SetVariables(x);

               parkMethod = 1;
           }
           parkAdjustTask.Init();
           break;
       case distanceTask:

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
//        if(ParkTask() == -1)
//            carStatus = stop;
        if(parkTask.Execute() == -1)
            carStatus = stop;
        break;
    case parkSim:
        carSim.Compute(-(motors[0].omega_actual + motors[1].omega_actual)/2*0.0325, 1);
        if(parkTask.Execute() == -1)
        {
            carStatus = stop;
        }
        break;
    case parkAdjust:
        carSim.Compute(-(motors[0].omega_actual + motors[1].omega_actual)/2*0.0325, 1);
        if(parkAdjustTask.Execute() == -1)
        {
            carStatus = stop;
        }
        break;
    case distanceTask:
        float err = (pidDistance[0].nowErr+pidDistance[1].nowErr)/2;
        if(err<=0.01 && err>=-0.01)
            carStatus = stop;
        break;
    default:
        break;
    }
}
PARK_PARAM parkParam = {0,0,0};
PARK_PARAM parkSimParam = {0,0,0};
uint8_t parkMethod = 1;
PID_PARAM parkInit = {0,0,0};
float parkMethodFloat = 0;
float parkInputs[3] = {0};

bool lastError[3] = {false};
int8_t Car::ParkTask_old()
{
    static int32_t step = 0;

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
        return FINAL_STEP;  //�������״̬

    switch(step)
    {
    case 0:
        //�ж��Ƿ�����ֹͣ����
        if(parkMethod == 0)
        {
            if(parkInputs[0]<=0.4 && parkInputs[0]>=0
                    && parkInputs[1]<=0.16 && parkInputs[1]>=0.06
                    && parkInputs[2]<=4 && parkInputs[2]>=-1.5)
            {
                ServoDirectionSet(0);
                return FINAL_STEP;
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
                return FINAL_STEP;
            }
            else
            {
                carSim.phi = fuzzy::perpendicular::CarFuzzy.Compute(parkInputs);
            }
        }
        if(parkMethod == 0)
            ServoDirectionSet(carSim.phi/32);
        else
            ServoDirectionSet(-carSim.phi/33);
        break;
    default:
        break;
    }
    return step;
}
int8_t Car::ParkAdjustTask()
{
    parkInputs[0] = parkSimParam.x;
    parkInputs[1] = parkSimParam.y;
    parkInputs[2] = parkSimParam.theta;

    if(parkMethod == 1)
    {
        if(parkInputs[0]<=0.19 && parkInputs[0]>=0.06
                && parkInputs[1]<=0.3 && parkInputs[1]>=0
                && parkInputs[2]<=92 && parkInputs[2]>=88)
        {
            ServoDirectionSet(0);
            return FINAL_STEP;
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
    ServoDirectionSet(-carSim.phi/27);
    return 0;
}

void Car::CommandQuery(uint32_t infraredCommand)
{
    switch(infraredCommand)
    {
    case 0x00FFA25D:    //1�ż�  ����
        carStatus = run;
        break;
    case 0x00FF629D:    //2�ż�  ֹͣ
        carStatus = stop;
        break;
    case 0x00FF22DD:    //4�ż� ɲ��
        carStatus = brake;
        break;
    case 0x00FF18E7:    //ǰ����
        carStatus = commandGoForward;
        break;
    case 0x00FF4AB5:    //���˼�
        carStatus = commandGoBack;
        break;
    case 0x00FF10EF:    //���
        carStatus = commandTurnLeft;
        break;
    case 0x00FF5AA5:    //�Ҽ�
        carStatus = commandTurnRight;
        break;
    case 0x00FF38C7:    //���ֱ��
        carStatus = commandGoStraight;
        break;
    case 0x00FFE21D:    //3��  Ѱ��
        carStatus = parkSim;
        parkMethod = 1;
        break;
    case 0x00FF02FD:    //5��  ƽ�в���
        carStatus = park;
        parkMethod = 0;
        break;
    case 0x00FFC23D:    //6��  ��ֱ����
        carStatus = park;
        parkMethod = 1;
        break;
    case 0x00FFE01F:    //7��  ������λ
        carStatus = parkAdjust;
        break;
    }
}
}
