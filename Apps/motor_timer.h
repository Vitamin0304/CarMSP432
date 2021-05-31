#ifndef APPS_MOTOR_TIMER_H_
#define APPS_MOTOR_TIMER_H_

#include <stdint.h>
#include <stdbool.h>


//void InfraredInit();
//void InfraredIntHandler();
void MotorPWMInit(float freq, float duty);
void MotorGPIOInit();
void MotorSpeedSet(float duty);
//void WTimer0Capture1Init();
//void WTimer5Capture0IntHandler();
//float WTimer5Capture0GetFrequency(uint32_t times);
void TA0_N_IRQHandler(void);
void InfraredInit();
void ServoInit();
void ServoDirectionSet(float direction);

extern uint32_t SysTickTime;
void SysTickInit();
void SysTick_Handler(void);

#endif /* APPS_MOTOR_TIMER_H_ */
