/*
 * Car.h
 *
 *  Created on: 2021��2��8��
 *      Author: ����
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

enum CarStatus
{
    stop = 0,
    run,
    brake,  //ɲ��
    commandGoForward,
    commandGoBack,
    //�������
    commandTurnRight,
    commandTurnLeft,
    commandGoStraight,
    //------
    park, //����
    parkSim,
    imageSeekLine,
    obstacle
};
extern volatile enum CarStatus carStatus;
//extern volatile enum MotorStatus previousStatus;

extern float defaultSpeed[2];
extern float parkSpeed[2];

const float carLength = 0.228f;
extern fuzzy::CarSim carSim;

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
    //����������
    static PARK_PARAM parkParam;
    static PARK_PARAM parkSimParam;
    static PID_PARAM parkInit;
    static float parkMethodFloat;
    static bool lastError[3];
    //float phi = 0;
private:
    Motor* motors;
    PIDSpeedController* pidSpeed;
    PIDDistanceController* pidDistance;
    float* nowSpeed = defaultSpeed;

    bool pidDistanceSwitch = false;
    //��������
    int8_t ParkTask();
    static uint8_t parkMethod;
};



#endif /* APPS_CAR_H_ */
