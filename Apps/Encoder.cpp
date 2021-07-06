/*
 * Encoder.cpp
 *
 *  Created on: 2021年1月25日
 *      Author: 电脑
 */
extern "C" {
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
}
#include <Apps/Encoder.h>

const float delay_s = 0.03;
//进入中断后变为false
volatile bool delayFlag;

uint32_t encoderCounter1[2] = {0};
uint32_t encoderCounter2[2] = {0};
int freqSign1 = 0;
int freqSign2 = 0;
float* pMotorFreq1;
float* pMotorFreq2;

uint32_t Aclk;
uint32_t SMclk;

//Timer 3.0
void CycleTimerInit()
{
    Aclk = CS_getACLK();
    SMclk = CS_getSMCLK();
    Timer_A_UpModeConfig upModeConfig =
    {
        TIMER_A_CLOCKSOURCE_ACLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        (uint16_t)(Aclk*delay_s),
        TIMER_A_TAIE_INTERRUPT_ENABLE,
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
        TIMER_A_SKIP_CLEAR
    };

    //计数设置
    Timer_A_configureUpMode(TIMER_A3_BASE, &upModeConfig);

    Interrupt_enableInterrupt(INT_TA3_N);
    Interrupt_disableInterrupt(INT_TA3_0);
    Interrupt_enableMaster();
    Timer_A_registerInterrupt(TIMER_A3_BASE, TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT, TA3_N_IRQHandler);

    Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);
}
void TA3_N_IRQHandler(void)
{
    Timer_A_clearInterruptFlag(TIMER_A3_BASE);
    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);


    *pMotorFreq1 = (encoderCounter1[0] + encoderCounter1[1])/3.0f/delay_s * freqSign1;
    encoderCounter1[0] = 0;
    encoderCounter1[1] = 0;
    *pMotorFreq2 = (encoderCounter2[0] + encoderCounter2[1])/3.0f/delay_s * freqSign2;
    encoderCounter2[0] = 0;
    encoderCounter2[1] = 0;
    delayFlag = false;
}
//编码器 P6.6 P6.7
void EncoderInit(float* pFreq1,float* pFreq2)
{
    pMotorFreq1 = pFreq1;
    pMotorFreq2 = pFreq2;

    Timer_A_CaptureModeConfig captureModeConfig =
    {
            TIMER_A_CAPTURECOMPARE_REGISTER_3,        // CC Register 2
            TIMER_A_CAPTUREMODE_RISING_EDGE,          // Rising Edge
            TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // CCIxB Input Select
            TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
            TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
            TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
    };
    Timer_A_initCapture(TIMER_A2_BASE, &captureModeConfig);
    captureModeConfig.captureMode = TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE;
    captureModeConfig.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    Timer_A_initCapture(TIMER_A2_BASE, &captureModeConfig);

    Interrupt_enableInterrupt(INT_TA2_N);
    //Interrupt_disableInterrupt(INT_TA2_0);
    Interrupt_enableMaster();
    Timer_A_registerInterrupt(TIMER_A2_BASE, TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT, TA2_N_IRQHandler);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN6,
            GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN7,
            GPIO_PRIMARY_MODULE_FUNCTION);


    Timer_A_initCapture(TIMER_A0_BASE, &captureModeConfig);
    captureModeConfig.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    captureModeConfig.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    Timer_A_initCapture(TIMER_A0_BASE, &captureModeConfig);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN6,
            GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN7,
            GPIO_PRIMARY_MODULE_FUNCTION);
}
void TA2_N_IRQHandler()
{
    static int8_t freqSignBefore1[3] = {0};

    uint32_t intStatus3 =
            Timer_A_getCaptureCompareInterruptStatus(
                    TIMER_A2_BASE,
                    TIMER_A_CAPTURECOMPARE_REGISTER_3,
                    TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG);
    uint32_t intStatus4 =
            Timer_A_getCaptureCompareInterruptStatus(
                    TIMER_A2_BASE,
                    TIMER_A_CAPTURECOMPARE_REGISTER_4,
                    TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG);

    if(intStatus3 == TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG)
    {
        Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,
                    TIMER_A_CAPTURECOMPARE_REGISTER_3);
        encoderCounter1[0]++;
        if(GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN7) == GPIO_INPUT_PIN_HIGH)
        {
            freqSign1 = 1;
        }
        else
        {
            freqSign1 = -1;
        }

        freqSignBefore1[0] = freqSignBefore1[1];
        freqSignBefore1[1] = freqSignBefore1[2];
        freqSignBefore1[2] = freqSign1;

//        if(freqSignBefore1[0] == 1 && freqSignBefore1[1] == 1)
        if(freqSignBefore1[0] == 1 && freqSignBefore1[1] == 1 && freqSignBefore1[2] == 1)
            freqSign1 = 1;
        else
            freqSign1 = -1;
    }
    if(intStatus4 == TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG)
    {
        Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,
                    TIMER_A_CAPTURECOMPARE_REGISTER_4);
        encoderCounter1[1]++;
    }
}


