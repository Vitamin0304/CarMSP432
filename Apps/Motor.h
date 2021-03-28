/*
 * Motor.h
 *
 *  Created on: 2021年1月25日
 *      Author: 电脑
 */

#ifndef APPS_MOTOR_H_
#define APPS_MOTOR_H_
extern "C" {
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
}


class Motor
{
public:
    Motor(uint32_t timer, uint_fast16_t timerRegister,
          uint_fast8_t pwmPort, uint_fast16_t pwmPin,
          uint_fast8_t gpioPort, uint_fast16_t gpioPin,
          float freq);
    void SetSpeed(float duty);
    void ComputeActualParam();

    virtual ~Motor();

    float freq_actual = 0;

    float omega_actual = 0;
    float omega_set = 0;

    float distance_actual = 0;
    float distance_set = 0;
private:
    Timer_A_PWMConfig pwmConfig;
    uint32_t timer;
    uint_fast8_t gpioPort;
    uint_fast16_t gpioPin;

    float nowSpeed;
};

#endif /* APPS_MOTOR_H_ */
