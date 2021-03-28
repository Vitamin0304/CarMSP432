/*
 * Encoder.h
 *
 *  Created on: 2021年1月25日
 *      Author: 电脑
 */

#ifndef APPS_ENCODER_H_
#define APPS_ENCODER_H_

extern volatile bool delayFlag;

extern const float delay_s;
extern uint32_t encoderCounter2[2];
extern int freqSign2;

void TA3_N_IRQHandler(void);
void CycleTimerInit();
void TA2_N_IRQHandler();

void EncoderInit(float* pFreq1,float* pFreq2);


#endif /* APPS_ENCODER_H_ */
