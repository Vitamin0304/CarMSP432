/*
 * Motor.cpp
 *
 *  Created on: 2021年1月25日
 *      Author: 电脑
 */
extern "C" {
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "Apps/my_uart.h"
}
#include <Apps/Motor.h>
#include <Apps/Encoder.h>
#include <Apps/motor_timer.h>
#include <cmath>

const float PI = 3.1415926;



const float defaultSpeed = 5;

const PID_PARAM pid_param_task1={0.05,0.05,0.005};
const PID_PARAM pid_param_task2={0.1,0.3,0.05};
const PID_PARAM pid_param_task3={0.11,0.3,0.1};

Motor::Motor(uint32_t timer, uint_fast16_t timerRegister,
             uint_fast8_t pwmPort, uint_fast16_t pwmPin,
             uint_fast8_t gpioPort, uint_fast16_t gpioPin,
             float freq)
{
    this->timer = timer;
    this->gpioPort = gpioPort;
    this->gpioPin = gpioPin;

    uint32_t SMclk = CS_getSMCLK();
    Timer_A_PWMConfig pwmConfig_temp =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        (uint16_t)(SMclk / freq),
        timerRegister,
        TIMER_A_OUTPUTMODE_SET_RESET,
        0
    };
    pwmConfig = pwmConfig_temp;

    Timer_A_generatePWM(timer, &pwmConfig);
    GPIO_setAsPeripheralModuleFunctionOutputPin(pwmPort, pwmPin,
                    GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setAsOutputPin(gpioPort, gpioPin);
}

void Motor::SetSpeed(float duty)
{
    if(duty >= 0)
    {
        pwmConfig.dutyCycle = pwmConfig.timerPeriod * duty;
        GPIO_setOutputHighOnPin(gpioPort, gpioPin);
    }
    else
    {
        pwmConfig.dutyCycle = pwmConfig.timerPeriod * (1+duty);
        GPIO_setOutputLowOnPin(gpioPort, gpioPin);
    }

    Timer_A_generatePWM(timer, &pwmConfig);
}
void Motor::ComputeActualParam()
{
    omega_actual = freq_actual / 390 * 2 * PI;
    distance_actual += omega_actual * 0.0325 * delay_s;
}
Motor::~Motor()
{
    // TODO Auto-generated destructor stub
}



