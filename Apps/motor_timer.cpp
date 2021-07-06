extern "C" {
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdio.h>
#include <string.h>

#include "Apps/my_uart.h"
}
#include <Apps/motor_timer.h>
#include <Apps/Motor.h>
#include <Apps/Encoder.h>
#include <Apps/Car.h>
#define SysCtlClock 48000000

uint32_t SysTickTime = 0;

Timer_A_PWMConfig pwmConfig =
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_1,
    SysCtlClock,
    TIMER_A_CAPTURECOMPARE_REGISTER_1,
    TIMER_A_OUTPUTMODE_SET_RESET,
    SysCtlClock
};

void SysTickInit()
{
    // Setting a Timer
    /* Configuring SysTick to trigger at 4800000 (MCLK is 48MHz so this will make
     * it toggle every 0.1s) */
    SysTick_enableModule();
    SysTick_setPeriod(CS_getMCLK()*0.001);
    Interrupt_disableSleepOnIsrExit();
    SysTick_registerInterrupt(SysTick_Handler);
    SysTick_enableInterrupt();
    // Enabling Master Interrupt
    Interrupt_enableMaster();
}

void SysTick_Handler(void)
{
    SysTickTime++;
}

void MotorPWMInit(float freq, float duty)
{
    pwmConfig.timerPeriod /= freq;
    pwmConfig.dutyCycle = pwmConfig.timerPeriod * duty;

    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
                GPIO_PRIMARY_MODULE_FUNCTION);
}
void MotorGPIOInit()
{
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4 | GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);
}
void MotorSpeedSet(float duty)
{
    if(duty >= 0)
    {
        pwmConfig.dutyCycle = pwmConfig.timerPeriod * duty;
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4);
    }
    else
    {
        pwmConfig.dutyCycle = pwmConfig.timerPeriod * (-duty);
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);
    }

    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
}

//红外遥控 P2.4
const uint16_t TIMER0_COUNT = 30000-1;
Timer_A_UpModeConfig upModeConfig =
{
    TIMER_A_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source
    TIMER_A_CLOCKSOURCE_DIVIDER_1,       // SMCLK/1 = 3MHz
    TIMER0_COUNT,
    TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
    TIMER_A_SKIP_CLEAR                   // Skup Clear Counter
};
Timer_A_CaptureModeConfig captureModeConfig =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_1,        // CC Register 2
    TIMER_A_CAPTUREMODE_RISING_EDGE,          // Rising Edge
    TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // CCIxB Input Select
    TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
    TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
    TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
};
Timer_A_CompareModeConfig compareModeConfig =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_2,
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
    TIMER_A_OUTPUTMODE_RESET_SET,
    4500-1
};
const float servoOffset = -0.05;
void ServoInit()
{
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5,
            GPIO_PRIMARY_MODULE_FUNCTION);
    Timer_A_initCompare(TIMER_A0_BASE,&compareModeConfig);
}
void ServoDirectionSet(float direction)
{
    if(direction > 1)
        direction = 1;
    else if(direction < -1)
        direction = -1;

    if(direction>0)
        direction*=1.8;
    else
        direction*=1.18;

    direction += servoOffset;
    uint16_t compareValue = 1500*direction + 4500;
    compareModeConfig.compareValue = compareValue;
    Timer_A_initCompare(TIMER_A0_BASE,&compareModeConfig);
}
void InfraredInit()
{
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN4,
            GPIO_PRIMARY_MODULE_FUNCTION);
    //捕获设置
    Timer_A_initCapture(TIMER_A0_BASE, &captureModeConfig);
    //计数设置
    Timer_A_configureUpMode(TIMER_A0_BASE, &upModeConfig);

    Interrupt_enableInterrupt(INT_TA0_N);
    Interrupt_enableMaster();
    Timer_A_registerInterrupt(TIMER_A0_BASE, TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT, TA0_N_IRQHandler);

    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}

void TA0_N_IRQHandler(void)
{
    uint32_t intStatus1 =
            Timer_A_getCaptureCompareInterruptStatus(
                    TIMER_A0_BASE,
                    TIMER_A_CAPTURECOMPARE_REGISTER_1,
                    TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG);
    uint32_t intStatus3 =
            Timer_A_getCaptureCompareInterruptStatus(
                    TIMER_A0_BASE,
                    TIMER_A_CAPTURECOMPARE_REGISTER_3,
                    TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG);
    uint32_t intStatus4 =
            Timer_A_getCaptureCompareInterruptStatus(
                    TIMER_A0_BASE,
                    TIMER_A_CAPTURECOMPARE_REGISTER_4,
                    TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG);

    static int8_t freqSignBefore2[3] = {0};

    static uint16_t ticks[2] = {0};
    static uint16_t tickCount = 0;
    uint16_t highTime = 0;
    static bool leaderCodeFlag = false;
    static uint32_t bits = 0;
    static uint32_t data = 0;
    uint8_t currentBit;

    if(intStatus1 == TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG)
    {
        char send[30] = {'\0'};

        Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,
                    TIMER_A_CAPTURECOMPARE_REGISTER_1);
        switch(tickCount)
        {
        case 0:
            captureModeConfig.captureMode = TIMER_A_CAPTUREMODE_FALLING_EDGE;//第一次捕获，改为下降沿
            Timer_A_initCapture(TIMER_A0_BASE, &captureModeConfig);
            ticks[tickCount++] = Timer_A_getCaptureCompareCount(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
            break;
        case 1:
            captureModeConfig.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;//第一次捕获，改为下降沿
            Timer_A_initCapture(TIMER_A0_BASE, &captureModeConfig);
            ticks[tickCount] = Timer_A_getCaptureCompareCount(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
            tickCount = 0;
            highTime = (ticks[0] < ticks[1]) ? (ticks[1] - ticks[0]) : (ticks[1] - ticks[0] + TIMER0_COUNT);
            highTime /= 3;  //单位us
    //        sprintf(send, "time: %d us\r\n",time);
    //        UART0SendString(send,strlen(send));
            break;
        }
        if(tickCount == 0)
        {
            if(highTime >= 4000 && highTime < 5000)    //4ms-5ms
            {
                data = 0;
                bits = 0;
                leaderCodeFlag = true;
                return;
            }
            else if(highTime >= 1200 && highTime < 3000)    //1.2ms-1.8ms
                currentBit = 1;
            else if(highTime >= 200 && highTime < 1000)     //0.2ms-1.0ms
                currentBit = 0;
            else
            {
                data = 0;
                bits = 0;
                leaderCodeFlag = false;
                return;
            }
            if(leaderCodeFlag)
            {
                bits++;
                data <<= 1;
                data |= currentBit;
            }
            if(bits==32)
            {
//                sprintf(send, "bits: %d\ndata: %0X\n",bits,data);
//                UART0SendString(send, strlen(send));
                carTask::Car::CommandQuery(data);
                //CarCommandQuery(data);
            }
        }
    }
    if(intStatus3 == TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG)
    {
        Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,
                    TIMER_A_CAPTURECOMPARE_REGISTER_3);
        encoderCounter2[0]++;
        if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7) == GPIO_INPUT_PIN_HIGH)
        {
            freqSign2 = -1;
        }
        else
        {
            freqSign2 = 1;
        }

        freqSignBefore2[0] = freqSignBefore2[1];
        freqSignBefore2[1] = freqSignBefore2[2];
        freqSignBefore2[2] = freqSign2;

        if(freqSignBefore2[0] == -1 && freqSignBefore2[1] == -1 && freqSignBefore2[2] == -1)
//        if(freqSignBefore2[0] == -1 && freqSignBefore2[1] == -1)
            freqSign2 = -1;
        else if(freqSignBefore2[0] == 1 && freqSignBefore2[1] == 1 && freqSignBefore2[2] == 1)
//        else if(freqSignBefore2[0] == 1 && freqSignBefore2[1] == 1)
            freqSign2 = 1;
    }
    if(intStatus4 == TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG)
    {
        Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,
                    TIMER_A_CAPTURECOMPARE_REGISTER_4);
        encoderCounter2[1]++;
    }
}
