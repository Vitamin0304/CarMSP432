/*
 * PIDController.h
 *
 *  Created on: 2021年1月29日
 *      Author: 电脑
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

    float Kp,Ki,Kd;  //pid系数
    float output = 0;    //输出
    float nowErr;    //当前偏差值
    float integral;  //积分值
    float lastErr;   //上一个偏差值
protected:
    float* pSetValue;  //设定值
    float* pActualValue;  //实际值
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
