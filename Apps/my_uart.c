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

#define MY_PI 3.1415926535897932384626

char uart0RxBuff[UART0_RX_BUFF_SIZE];
char uart1RxBuff[UART1_RX_BUFF_SIZE];
char uart2RxBuff[UART2_RX_BUFF_SIZE];
uint32_t uart0RxLength = 0;
uint32_t uart1RxLength = 0;
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

void UART1Init(PARK_PARAM* pReverseParam_temp)
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
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,
                    GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    UART_initModule(EUSCI_A1_BASE, &uartConfig);

    UART_enableModule(EUSCI_A1_BASE);

    UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA1);
//    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster();
}
bool parkParamIsNew = false;
void EUSCIA1_IRQHandler(void)
{
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A1_BASE);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        uint8_t rxChar = UART_receiveData(EUSCI_A1_BASE);
        if(receivePiData(rxChar) == -1)
        {
            if(verify(uart1RxBuff,uart1RxLength-1))
            {
                uart1RxLength -= 2;
                uart1RxBuff[uart1RxLength++] = '\r';
                uart1RxBuff[uart1RxLength++] = '\n';
                uart1RxBuff[uart1RxLength] = '\0';

                cJSON* root = cJSON_Parse(uart1RxBuff);
                if(root != 0) //转换成功
                {
                    if(cJSON_GetArraySize(root) == 4)
                    {
                        pReverseParam->x = cJSON_GetArrayItem(root,0)->valuedouble;
                        pReverseParam->y = cJSON_GetArrayItem(root,1)->valuedouble;
                        pReverseParam->theta = cJSON_GetArrayItem(root,2)->valuedouble;
                        pReverseParam->flag = cJSON_GetArrayItem(root,3)->valuedouble;
                        if(pReverseParam->flag == 1)
                            parkParamIsNew = true;
                    }
                    cJSON_Delete(root);
                }
            }
            uart1RxLength = 0;
        }
    }
}

float IMU_Angle[3] = {0};
void UART2Init()
{
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
inline float mod2pi(float x)
{
    if (x > MY_PI)
        x -= 2 * MY_PI;
    else if (x < -MY_PI)
        x += 2 * MY_PI;
    return x;
}
void EUSCIA2_IRQHandler(void)
{
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        char rxChar = UART_receiveData(EUSCI_A2_BASE);
        if(receiveIMUData(rxChar) == -1)
        {
            IMU_Angle[0] = mod2pi((float)((short)uart2RxBuff[3]<<8|uart2RxBuff[2]) *MY_PI/ 32768);
            IMU_Angle[1] = mod2pi((float)((short)uart2RxBuff[5]<<8|uart2RxBuff[4]) *MY_PI/ 32768);
            IMU_Angle[2] = mod2pi((float)((short)uart2RxBuff[7]<<8|uart2RxBuff[6]) *MY_PI/ 32768);
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

int receivePiData(uint8_t rxChar)
{
    static int32_t step = 0;
    switch(step)
    {
    case 0:
        if(rxChar == '[')
        {
            uart1RxBuff[uart1RxLength++] = rxChar;
            step++;
        }
        break;
    case 1:
        uart1RxBuff[uart1RxLength++] = rxChar;
        if(rxChar == 0xA5)
        {
            step = 0;
            return -1;
        }
        break;
    }
    return step;
}
uint8_t sum = 0;
bool verify(uint8_t* str, uint32_t len)
{
    sum = 0;
    for(int i = 0; i < len-1; ++i)
    {
        sum += str[i];
    }
    if(str[len-1] == sum)
        return true;
    else
        return false;
}

int receiveIMUData(char rxChar)
{
    static int32_t step = 0;
    switch(step)
    {
    case 0:
        if(rxChar == 0x55)
        {
            uart2RxLength = 0;
            uart2RxBuff[uart2RxLength++] = rxChar;
            step++;
        }
        break;
    case 1: //姿态角信息
        if(rxChar == 0x53)
        {
            uart2RxBuff[uart2RxLength++] = rxChar;
            step++;
        }
        else
            step = 0;
        break;
    case 2:
        uart2RxBuff[uart2RxLength++] = rxChar;
        if(uart2RxLength == 11)
        {
            uint8_t sum = 0;
            for(int i=0; i < 10; ++i)
            {
                sum += uart2RxBuff[i];
            }
            if(sum == uart2RxBuff[10]) //接收成功
            {
                step = 0;
                return -1;
            }
            else
            {
                step = 0;
                return 2;
            }
        }
        break;
    }
    return step;
}

void sendIMUcmd(int cmd)
{
    char send[3] = {0xFF,0xAA,0};
    switch(cmd)
    {
    case 0: //角度初始化
        send[2] = 0x52;
        break;
    case 1: //加速度计校准
        send[2] = 0x67;
        break;
    default:
        return;
        break;
    }
    for (int i = 0; i < 3; ++i)
    {
        UART_transmitData(EUSCI_A2_BASE, send[i]);
    }
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
    sprintf(send,"[%.2f,%.6f,%.6f,%.6f,%.6f]",time,data[0],data[1],data[2],data[3]);
    for (int i = 0; i < 60 && send[i] != '\0'; ++i)
    {
        UART_transmitData(EUSCI_A0_BASE, send[i]);
    }
}
