#include <Apps/Motor.h>
#include <Apps/PIDController.h>
#include <Apps/Encoder.h>
#include <cmath>
#define PI 3.1415926f

PIDController::PIDController(float* pActualValue, float* pSetValue, float Kp, float Ki, float Kd)
{
    this->pActualValue = pActualValue;
    this->pSetValue = pSetValue;
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->lastErr = 0;
    this->integral = 0;
}
PIDController::~PIDController()
{
    // TODO Auto-generated destructor stub
}
void PIDController::Reset()
{
    this->lastErr = 0;
    this->integral = 0;
}
void PIDController::SetPIDParam(PID_PARAM* pidParam)
{
    this->Kp = pidParam->Kp;
    this->Ki = pidParam->Ki;
    this->Kd = pidParam->Kd;
}
PIDSpeedController::PIDSpeedController(Motor* pMotor, PID_PARAM* pidParam)
    :PIDController(&(pMotor->omega_actual),&(pMotor->omega_set),
                   pidParam->Kp,pidParam->Ki,pidParam->Kd)
{
}
float PIDSpeedController::a = 0.5;
float PIDSpeedController::Compute()
{
    omega_revised = a * lastActualValue + (1-a)*(*pActualValue);

    //±àÂëÆ÷¾À´í
    if((lastActualValue-*pActualValue > 8 || lastActualValue-*pActualValue < -8)
            && lastActualValue*(*pActualValue)<0)
        *pActualValue = - *pActualValue;

    if(omega_revised > 60)
        omega_revised = *pSetValue - lastErr;

    float P = 0, I = 0, D = 0;
    nowErr = *pSetValue - omega_revised;

    P = Kp * nowErr;

//    if(nowErr>-10 && nowErr<10)
    {
        integral += (nowErr+lastErr)/2;
        I = Ki * integral *delay_s;
        if(integral<-1000)
            integral = -1000;
        else if(integral>1000)
            integral = 1000;
    }
    //if((nowErr - lastErr)<-1 || (nowErr - lastErr)>1)
        D = Kd * (nowErr - lastErr) /delay_s;

    lastErr = nowErr;
    lastActualValue = omega_revised;

    output = P + I + D;
    if(output > 0.98)
        output = 0.98;
    else if(output < -0.98)
        output = -0.98;

    return output;
}

PIDDistanceController::PIDDistanceController(Motor* pMotor, PID_PARAM* pidParam)
    :PIDController(&(pMotor->distance_actual),&(pMotor->distance_set),
               pidParam->Kp,pidParam->Ki,pidParam->Kd)
{
}
float PIDDistanceController::Compute()
{
    float P=0, I=0, D=0;
    nowErr = *pSetValue - *pActualValue;


        P = Kp * nowErr;

        integral += (nowErr+lastErr)/2;
        I = Ki * integral *delay_s;
        if(integral<-1000)
            integral = -1000;
        else if(integral>1000)
            integral = 1000;

//    if((nowErr - lastErr)<-1 || (nowErr - lastErr)>1)
        D = Kd * (nowErr - lastErr) /delay_s;


    lastErr = nowErr;

    output = P + I + D;
    if(output > outputMax)
        output = outputMax;
    else if(output < -outputMax)
        output = -outputMax;

    return output;
}
