
extern "C" {
#include "msp.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "Apps/my_uart.h"
#include "OLED/MyOLED.h"
}
#include <Apps/motor_timer.h>
#include <Apps/Motor.h>
#include <Apps/Encoder.h>
#include <Apps/PIDController.h>
#include <Apps/Car.h>
#include <FuzzyControl/CarFuzzy.h>

//#include <Eigen/Core>
//#include <PnP/SolvePnP.h>

//using namespace Eigen;

void ClockInit();

//const float PI = 3.1415926;

void main(void)
{
    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    ClockInit();

    OLED_Init();
    TimeUsed_Display(10, 23, 1);

    PID_PARAM PIDMotorParam = {0.1,0.1,0};
    PID_PARAM PIDDistanceParam = {12,0.8,0.8};
    UART0Init(&Car::parkInit,&Car::parkMethodFloat);

    UART2Init(&Car::parkParam);

    InfraredInit();
    ServoInit();

    Motor motors[2] = {
        Motor(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1,
             GPIO_PORT_P5, GPIO_PIN6,
             GPIO_PORT_P5, GPIO_PIN4,
             10000),
        Motor(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2,
            GPIO_PORT_P5, GPIO_PIN7,
            GPIO_PORT_P5, GPIO_PIN5,
            10000)
    };
    motors[0].SetSpeed(0);
    motors[1].SetSpeed(0);

//    CarFuzzyInit();
    float fuzzyInput[3];
    float fuzzyOutput;

    EncoderInit(&(motors[0].freq_actual),&(motors[1].freq_actual));

    PIDSpeedController pidSpeed[2] = {
        PIDSpeedController(&motors[0],&PIDMotorParam),
        PIDSpeedController(&motors[1],&PIDMotorParam)
    };
    PIDDistanceController pidDistance[2] = {
        PIDDistanceController(&motors[0],&PIDDistanceParam),
        PIDDistanceController(&motors[1],&PIDDistanceParam)
    };

    Car car(motors, pidSpeed, pidDistance);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    CycleTimerInit();

    float sendData[4] = {0};
    uint32_t times = 0;

//    Matrix<double,3,3> pixel;
//    pixel<< 209, 370, 510, 339, 190, 373, 282, 135;
//
//    Vector2d F(0,0);
//
//    SolvePnP(pixel, F);


    while(1)
    {
        pidSpeed[0].SetPIDParam(&PIDMotorParam);
        pidSpeed[1].SetPIDParam(&PIDMotorParam);
        pidDistance[0].SetPIDParam(&PIDDistanceParam);
        pidDistance[1].SetPIDParam(&PIDDistanceParam);

        motors[0].ComputeActualParam();
        motors[1].ComputeActualParam();

        car.StatusManage();
        car.ExecuteTask();
        car.PIDCompute();

//        sendData[0] = motors[0].omega_set;
//        sendData[1] = car.motor_output[0];
//        sendData[2] = pidSpeed[0].omega_revised;
//        sendData[3] = pidSpeed[0].integral;

        sendData[0] = Car::parkParam.x;
        sendData[1] = Car::parkParam.y;
        sendData[2] = Car::parkParam.theta;
        sendData[3] = carSim.phi;

        times++;
        //if(mqttConnected == true && times%6 == 0)
        if(times%2 == 0)
        {
            sendChartDataArray(times*delay_s,sendData);
            //sendMatlabArray(times*delay_s,sendData);
        }

        delayFlag = true;
        while(delayFlag);
    }
}

void ClockInit()
{
    /* Before we start we have to change VCORE to 1 to support the 48MHz frequency */
    PCM_setCoreVoltageLevel(PCM_AM_LDO_VCORE1);
    FlashCtl_setWaitState(FLASH_BANK0, 1);
    FlashCtl_setWaitState(FLASH_BANK1, 1);

    /* Setting DCO to 12MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);

    CS_initClockSignal(CS_MCLK,  CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); // 48000000 Hz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_16); //  3000000 Hz
//
//    CS_setExternalClockSourceFrequency(32000, 48000000);
    CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
//    CS_initClockSignal(CS_BCLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
}
