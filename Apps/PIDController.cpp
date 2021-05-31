#include <Apps/Motor.h>
#include <Apps/PIDController.h>
#include <Apps/Encoder.h>
#include <cmath>
#define PI 3.1415926f

inline float Saturation(float input, float max)
{
    if(input > max)
        return max;
    else if(input < -max)
        return -max;
    else
        return input;
}

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

    //编码器纠错
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
ESOSpeedController::ESOSpeedController(Motor* pMotor, PID_PARAM* pidParam)
    :PIDController(&(pMotor->omega_actual),&(pMotor->omega_set),
                   pidParam->Kp,pidParam->Ki,pidParam->Kd)
{
}
inline float sign(float x)
{
    if(x > 0)
        return 1;
    else if(x == 0)
        return 0;
    else
        return -1;
}
inline float Fal(float e,float alpha)
{
    float delta = 0.05;
    if(abs(e)>delta)
        return pow(abs(e),alpha)*sign(e);
    else
        return e/pow(delta,1-alpha);
}
inline float Fst(float v0,float v1,float v2,float r,float h)
{
    float d = r*h;
    float d0 = d*h;
    float y = v1 - v0 + h*v2;
    float a0 = pow(d*d+8*r*abs(y),0.5);
    float a;
    if(abs(y)>d0)
        a = v2 + y/h;
    else
        a = v2 + (sign(y)*(a0-d))/2;
    if(a<=d)
        return -r*a;
    else
        return -r*sign(a);
}
float ESOSpeedController::b = 800;
float ESOSpeedController::Compute()
{
    //编码器纠错
    if((lastActualValue-*pActualValue > 8 || lastActualValue-*pActualValue < -8)
            && lastActualValue*(*pActualValue)<0)
        *pActualValue = - *pActualValue;

    nowErr = z1 - *pActualValue;

    float w0 = Ki;
    float w02 = w0*w0;
    float w03 = w02*w0;

    //前向积分
//    z1 += (z2 - 3*w0 *Fal(nowErr,1)) * delay_s;
//    z2 += (z3 - 3*w02/5 *Fal(nowErr,0.5) + b*output) * delay_s;
//    z3 += (-w03/9 *Fal(nowErr,0.3)) * delay_s;

    //后向积分
    float dotz[3];
    dotz[2] = -w03 *Fal(nowErr,0.3);
    z3 += dotz[2]* delay_s;
    dotz[1] = z3 - 3*w02 *Fal(nowErr,0.5) + b*output;
    z2 += dotz[1]* delay_s;
    dotz[0] = z2 - 3*w0 *Fal(nowErr,1);
    z1 += dotz[0]* delay_s;

    //梯形积分
//    float dotz[3];
//    dotz[2] = -w03/9 *Fal(nowErr,0.3);
//    z3 += (dotz[2]+lastdotz[2]) /2 * delay_s;
//    dotz[1] = z3 - 3*w02/5 *Fal(nowErr,0.5) + b*output;
//    z2 += (dotz[1]+lastdotz[1]) /2 * delay_s;
//    dotz[0] = z2 - 3*w0 *Fal(nowErr,1);
//    z1 += (dotz[0]+lastdotz[0]) /2 * delay_s;

//    lastdotz[0] = dotz[0];
//    lastdotz[1] = dotz[1];
//    lastdotz[2] = dotz[2];

    //TD
//    v2 += Fst(*pSetValue,v1,v2,40,0.2) * delay_s;
//    v1 += v2 * delay_s;
//    float wt = 50;
//    v2 += (wt*wt*(*pSetValue-v1) - 2*wt*v2) * delay_s;
//    v1 += v2 * delay_s;

    output = Kp * Fal((*pSetValue - z1),0.8) + Kd * Fal(Saturation(z2,25),1.1) - z3/b;
//    output = Kp * Fal((v1 - z1),1) + Kd * Fal(z2,1) - z3/b;


    lastActualValue = *pActualValue;

    if(output > 0.95)
        return 0.95;
    else if(output < -0.95)
        return -0.95;
    else
        return output;
//    if(output > 0.98)
//        output = 0.98;
//    else if(output < -0.98)
//        output = -0.98;
//
//    return output;
}
void ESOSpeedController::Reset()
{
    z1 = 0;
    z2 = 0;
    z3 = 0;
    v1 = 0;
    v2 = 0;
    output = 0;
    lastActualValue = 0;
    lastdotz[0] = 0;
    lastdotz[1] = 0;
    lastdotz[2] = 0;
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
float parkAdjustSet = 0;
PIDParkAdjustController::PIDParkAdjustController(float* pActualValue, PID_PARAM* pidParam)
    :PIDController(pActualValue,&parkAdjustSet,
               pidParam->Kp,pidParam->Ki,pidParam->Kd)
{
}
float PIDParkAdjustController::Compute()
{
    float P=0, I=0, D=0;
    nowErr = pActualValue[0]-0.125;

    P = Kp * nowErr;

    integral += (nowErr+lastErr)/2;
    I = Ki * integral *delay_s;
    if(integral<-1000)
        integral = -1000;
    else if(integral>1000)
        integral = 1000;

//    if((nowErr - lastErr)<-1 || (nowErr - lastErr)>1)
    if(firstCompute == false)
        D = Kd * (nowErr - lastErr) /delay_s;

    firstCompute = false;

    lastErr = nowErr;

    output = P + I + D;
//    if(output > 25)
//        output = 25;
//    else if(output < -25)
//        output = -25;

    return output;
}
PIDPathTrackController::PIDPathTrackController(float* pActualValue, PID_PARAM* pidParam)
    :PIDController(pActualValue,&parkAdjustSet,
               pidParam->Kp,pidParam->Ki,pidParam->Kd)
{
}
float PIDPathTrackController::Compute()
{
    float P=0, I=0, D=0;
    nowErr = *pActualValue;

    P = Kp * nowErr;

    switch(out_max_flag)
    {
    case -1:
        if(nowErr > 0)
            integral += (nowErr+lastErr)/2;
        break;
    case 0:
        integral += (nowErr+lastErr)/2;
        break;
    case 1:
        if(nowErr < 0)
            integral += (nowErr+lastErr)/2;
        break;
    }
    I = Ki * integral *delay_s;

//    if((nowErr - lastErr)<-1 || (nowErr - lastErr)>1)
    D = Kd * (nowErr - lastErr) /delay_s;
//    D = (nowErr - exp(-delay_s/Kd)*lastErr + exp(-delay_s/0.15/Kd)*lastOutput)/0.15;


    output = P + I + D;

    const float phi_max = 28 *3.1415926/180;
    if (output > phi_max)
    {
        output = phi_max;
        out_max_flag = 1;
    }
    else if (output < -phi_max)
    {
        output = -phi_max;
        out_max_flag = -1;
    }
    else
        out_max_flag = 0;

    lastOutput = output;
    lastErr = nowErr;

    return output;
}
