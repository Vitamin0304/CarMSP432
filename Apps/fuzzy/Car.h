/*
 * Car.h
 *
 *  Created on: 2021年2月8日
 *      Author: 电脑
 */

#ifndef APPS_CAR_H_
#define APPS_CAR_H_

extern "C"
{
    #include "my_uart.h"
}
#include <Apps/PIDController.h>
#include <FuzzyControl/CarFuzzy.h>
#include <FuzzyControl/CarSim.h>
#include <Apps/Motor.h>

#define FINAL_STEP -1

namespace carTask
{
enum CarStatus
{
    stop = 0,
    run,
    brake,  //刹车
    commandGoForward,
    commandGoBack,
    //舵机操作
    commandTurnRight,
    commandTurnLeft,
    commandGoStraight,
    //------
    park, //倒车
    parkSim,
    parkAdjust,
    distanceTask
};
extern volatile enum CarStatus carStatus;
//extern volatile enum MotorStatus previousStatus;

extern float defaultSpeed[2];
extern float parkSpeed[2];

const float carLength = 0.228f;
extern fuzzy::CarSim carSim;

extern PARK_PARAM parkParam;
extern PARK_PARAM parkSimParam;
extern PID_PARAM parkInit;
extern float parkMethodFloat;
extern bool lastError[3];
//float phi = 0;
extern float parkInputs[3];
extern float* nowSpeed;

extern bool pidDistanceSwitch;
extern uint8_t parkMethod;
extern PIDParkAdjustController* pidParkAdjust;

class Car
{
public:
    Car(Motor* motors,
        PIDSpeedController* pidSpeed,
        PIDDistanceController* pidDistance);
    virtual ~Car();

    static void CommandQuery(uint32_t infraredCommand);
    void StatusManage();

    void MotorRun();
    void MotorStop();
    void MotorGoBack();

    void SetDistance(float distance);

    void PIDCompute();
    void ExecuteTask();

    float omega_set_output[2] = {0};
    float motor_output[2] = {0};

    PIDDistanceController* pidDistance;
private:
    Motor* motors;
    PIDSpeedController* pidSpeed;


    //倒车函数
    int8_t ParkTask_old();
    int8_t ParkAdjustTask();
};

extern Car* _car;

}

#endif /* APPS_CAR_H_ */
