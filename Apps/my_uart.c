/*
 * my_uart.c
 *
 *  Created on: 2021年1月24日
 *      Author: 电脑
 */
#include "ti/devices/msp432p4xx/driverlib/driverlib.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "Apps/my_uart.h"
#include "cJSON.h"

char uart0RxBuff[UART0_RX_BUFF_SIZE];
char uart2RxBuff[UART2_RX_BUFF_SIZE];
uint32_t uart0RxLength = 0;
uint32_t uart2RxLength = 0;

bool mqttConnected = false;

PID_PARAM* pPIDParam;
float* param4;

void UART0Init(PID_PARAM* pPIDParam_temp,float* param4_temp)
{
    pPIDParam = pPIDParam_temp;
    param4 = param4_temp;

    const eUSCI_UART_ConfigV1 uartConfig =
    {
            EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
            1,                                     // BRDIV = 78
            10,                                       // UCxBRF = 2
            0,                                       // UCxBRS = 0
            EUSCI_A_UART_NO_PARITY,                  // No Parity
            EUSCI_A_UART_LSB_FIRST,                  // LSB First
            EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
            EUSCI_A_UART_MODE,                       // UART mode
            EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
            EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
    };
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                    GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    UART_initModule(EUSCI_A0_BASE, &uartConfig);

    UART_enableModule(EUSCI_A0_BASE);

    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0);
//    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster();
}
void UART0SendString(char* str, uint32_t len)
{
    for(int i = 0; i < len; ++i)
    {
        UART_transmitData(EUSCI_A0_BASE, str[i]);
    }
}

void EUSCIA0_IRQHandler(void)
{
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        char rxChar = UART_receiveData(EUSCI_A0_BASE);
        if(receiveSettingsData(rxChar) == -1)
        {
            uart0RxBuff[uart0RxLength++] = '\r';
            uart0RxBuff[uart0RxLength++] = '\n';
            uart0RxBuff[uart0RxLength] = '\0';
            if(uart0RxLength == 8 && uart0RxBuff[1]=='M' && uart0RxBuff[2]=='Q')
                mqttConnected = true;

            cJSON* root = cJSON_Parse(uart0RxBuff);
            if(root != 0) //转换成功
            {
                if(cJSON_GetArraySize(root) == 4)
                {
                    pPIDParam->Kp = cJSON_GetArrayItem(root,0)->valuedouble;
                    pPIDParam->Ki = cJSON_GetArrayItem(root,1)->valuedouble;
                    pPIDParam->Kd = cJSON_GetArrayItem(root,2)->valuedouble;
                    *param4 = cJSON_GetArrayItem(root,3)->valuedouble;
                }
                cJSON_Delete(root);
            }

//            UARTprintf("%s",uart0RxBuff);
            uart0RxLength = 0;
        }
    }
}

PARK_PARAM* pReverseParam;

void UART2Init(PARK_PARAM* pReverseParam_temp)
{
    pReverseParam = pReverseParam_temp;

    const eUSCI_UART_ConfigV1 uartConfig =
    {
            EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
            1,                                     // BRDIV = 78
            10,                                       // UCxBRF = 2
            0,                                       // UCxBRS = 0
            EUSCI_A_UART_NO_PARITY,                  // No Parity
            EUSCI_A_UART_LSB_FIRST,                  // LSB First
            EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
            EUSCI_A_UART_MODE,                       // UART mode
            EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
            EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
    };
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                    GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    UART_initModule(EUSCI_A2_BASE, &uartConfig);

    UART_enableModule(EUSCI_A2_BASE);

    UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA2);
//    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster();
}
void EUSCIA2_IRQHandler(void)
{
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        char rxChar = UART_receiveData(EUSCI_A2_BASE);
        if(receivePiData(rxChar) == -1)
        {
            uart2RxBuff[uart2RxLength++] = '\r';
            uart2RxBuff[uart2RxLength++] = '\n';
            uart2RxBuff[uart2RxLength] = '\0';

            cJSON* root = cJSON_Parse(uart2RxBuff);
            if(root != 0) //转换成功
            {
                if(cJSON_GetArraySize(root) == 4)
                {
                    pReverseParam->x = cJSON_GetArrayItem(root,0)->valuedouble;
                    pReverseParam->y = cJSON_GetArrayItem(root,1)->valuedouble;
                    pReverseParam->theta = cJSON_GetArrayItem(root,2)->valuedouble;
                    pReverseParam->flag = cJSON_GetArrayItem(root,3)->valuedouble;
                }
                cJSON_Delete(root);
            }
            uart2RxLength = 0;
        }
    }
}

int receiveSettingsData(char rxChar)
{
    static int32_t step = 0;
    switch(step)
    {
    case 0:
        if(rxChar == '[')
        {
            uart0RxBuff[uart0RxLength++] = rxChar;
            step++;
        }
        break;
    case 1:
        uart0RxBuff[uart0RxLength++] = rxChar;
        if(rxChar == ']')
        {
            step = 0;
            return -1;
        }
        break;
    }
    return step;
}

int receivePiData(char rxChar)
{
    static int32_t step = 0;
    switch(step)
    {
    case 0:
        if(rxChar == '[')
        {
            uart2RxBuff[uart2RxLength++] = rxChar;
            step++;
        }
        break;
    case 1:
        uart2RxBuff[uart2RxLength++] = rxChar;
        if(rxChar == ']')
        {
            step = 0;
            return -1;
        }
        break;
    }
    return step;
}
void sendChartData(float time,float data1,float data2,float data3,float data4)
{
    char send[60];
    sprintf(send,"{\"time\":%.2f,\"data\":[%.3f,%.3f,%.3f,%.3f]}\r\n",time,data1,data2,data3,data4);
    for (int i = 0; i < 60 && send[i] != '\0'; ++i)
    {
        UART_transmitData(EUSCI_A0_BASE, send[i]);
    }
}
void sendChartDataArray(float time,float* data)
{
    char send[60];
    sprintf(send,"{\"time\":%.2f,\"data\":[%.3f,%.3f,%.3f,%.3f]}\r\n",time,data[0],data[1],data[2],data[3]);
    for (int i = 0; i < 60 && send[i] != '\0'; ++i)
    {
        UART_transmitData(EUSCI_A0_BASE, send[i]);
    }
}

void sendMatlabArray(float time,float* data)
{
    char send[60];
    sprintf(send,"[%.2f,%.3f,%.3f,%.3f,%.3f]",time,data[0],data[1],data[2],data[3]);
    for (int i = 0; i < 60 && send[i] != '\0'; ++i)
    {
        UART_transmitData(EUSCI_A0_BASE, send[i]);
    }
}
