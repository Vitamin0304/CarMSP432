/*
 * PIDController.h
 *
 *  Created on: 2021��1��29��
 *      Author: ����
 */

#ifndef APPS_PIDCONTROLLER_H_
#define APPS_PIDCONTROLLER_H_

extern "C"
{
#include "Apps/my_uart.h"
}
#include <Apps/Motor.h>
#define FINAL_STEP       -1

class PIDController
{
public:
    PIDController(float* pActualValue, float* pSetValue, float Kp, float Ki, float Kd);
    void Reset();
    void SetPIDParam(PID_PARAM* pidParam);
    virtual float Compute() = 0;
    virtual ~PIDController();

    float Kp,Ki,Kd;  //pidϵ��
    float output = 0;    //���
    float nowErr;    //��ǰƫ��ֵ
    float integral;  //����ֵ
    float lastErr;   //��һ��ƫ��ֵ
protected:
    float* pSetValue;  //�趨ֵ
    float* pActualValue;  //ʵ��ֵ
};

class PIDSpeedController : public PIDController
{
public:
    PIDSpeedController(Motor* pMotor, PID_PARAM* pidParam);
    float Compute();
    float lastOutput = 0;
    static float Td;
private:
    float lastActualValue = 0;
    int out_max_flag = 0;
};

class ESOSpeedController : public PIDController
{
public:
    ESOSpeedController(Motor* pMotor, PID_PARAM* pidParam);
    float Compute();
    void Reset();

    float z1 = 0;
    float z2 = 0;
    float z3 = 0;

    float v1 = 0;
    float v2 = 0;

    static float b;
private:
    float lastActualValue = 0;
    float lastdotz[3] = {0};
};
class PIDDistanceController : public PIDController
{
public:
    PIDDistanceController(Motor* pMotor, PID_PARAM* pidParam);
    float Compute();
    float outputMax;
};
class PIDParkAdjustController : public PIDController
{
public:
    PIDParkAdjustController(float* pActualValue, PID_PARAM* pidParam);
    float Compute();
    bool firstCompute = true;
};

class PIDPathTrackController : public PIDController
{
public:
    PIDPathTrackController(float* pActualValue, PID_PARAM* pidParam);
    float Compute();
private:
    int out_max_flag = 0;
    float lastOutput = 0;
};

#endif /* APPS_PIDCONTROLLER_H_ */
