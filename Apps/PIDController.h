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
    float output;    //输出
    float nowErr;    //当前偏差值
    float integral;  //积分值
protected:
    float* pSetValue;  //设定值
    float* pActualValue;  //实际值

    float lastErr;   //上一个偏差值
};

class PIDSpeedController : public PIDController
{
public:
    PIDSpeedController(Motor* pMotor, PID_PARAM* pidParam);
    float Compute();
    float omega_revised;
    static float a;
private:
    float lastActualValue = 0;
};
class PIDDistanceController : public PIDController
{
public:
    PIDDistanceController(Motor* pMotor, PID_PARAM* pidParam);
    float Compute();
    float outputMax;
};


#endif /* APPS_PIDCONTROLLER_H_ */
