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
    float output;    //���
    float nowErr;    //��ǰƫ��ֵ
    float integral;  //����ֵ
protected:
    float* pSetValue;  //�趨ֵ
    float* pActualValue;  //ʵ��ֵ

    float lastErr;   //��һ��ƫ��ֵ
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
