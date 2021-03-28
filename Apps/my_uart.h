/*
 * my_uart.h
 *
 *  Created on: 2021年1月24日
 *      Author: 电脑
 */

#ifndef APPS_MY_UART_H_
#define APPS_MY_UART_H_
#include <stdint.h>
#define UART0_RX_BUFF_SIZE  100
#define UART2_RX_BUFF_SIZE  100

extern bool mqttConnected;

typedef struct pid_param
{
    float Kp;
    float Ki;
    float Kd;
}PID_PARAM;

typedef struct park_param
{
    float x;
    float y;
    float theta;
    uint8_t flag;
}PARK_PARAM;

void UART0Init(PID_PARAM* pPIDParam_temp,float* param4_temp);
void EUSCIA0_IRQHandler(void);

void UART2Init(PARK_PARAM* pReverseParam_temp);
void EUSCIA2_IRQHandler(void);

void UART0SendString(char* str, uint32_t len);
int receiveSettingsData(char rxChar);
int receivePiData(char rxChar);
void sendChartData(float time,float data1,float data2,float data3,float data4);
void sendChartDataArray(float time,float* data);
void sendMatlabArray(float time,float* data);

#endif /* APPS_MY_UART_H_ */
