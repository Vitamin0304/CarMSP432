
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
#include <PathTrack/PathTrackSim.h>

//#include <Eigen/Core>
//#include <PnP/SolvePnP.h>

//using namespace Eigen;

void ClockInit();

//const float PI = 3.1415926;

int main(void)
{
    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    ClockInit();

    SysTickInit();

    OLED_Init();
    TimeUsed_Display(10, 23, 1);

    PID_PARAM PIDMotorParam = {0.16,0.6,0.002};
    PID_PARAM PIDDistanceParam = {15,0.8,0.8};

    PID_PARAM PIDParkAdjustParam = {60, 0, 20};

//    PID_PARAM ESOMotorParam = {0.16,14,0.008};
    PID_PARAM ESOMotorParam = {0.12,14,0.01};

//    UART0Init(&PIDParkAdjustParam,&Car::parkMethodFloat);
    UART0Init(&carTask::parkInit,&carTask::parkMethodFloat);
//    UART0Init(&ESOMotorParam,&ESOSpeedController::b);
//    UART0Init(&PIDMotorParam,&PIDSpeedController::Td);
    UART1Init(&carTask::parkParam);
    UART2Init();

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

    EncoderInit(&(motors[0].freq_actual),&(motors[1].freq_actual));

    PIDSpeedController pidSpeed[2] = {
        PIDSpeedController(&motors[0],&PIDMotorParam),
        PIDSpeedController(&motors[1],&PIDMotorParam)
    };
    ESOSpeedController ESOSpeed[2] = {
        ESOSpeedController(&motors[0],&ESOMotorParam),
        ESOSpeedController(&motors[1],&ESOMotorParam)
    };
    PIDDistanceController pidDistance[2] = {
        PIDDistanceController(&motors[0],&PIDDistanceParam),
        PIDDistanceController(&motors[1],&PIDDistanceParam)
    };

    carTask::Car car(motors, pidSpeed, ESOSpeed, pidDistance);
    carTask::_car = &car;

    PIDParkAdjustController pidParkAdjust(carTask::parkInputs,&PIDParkAdjustParam);
    carTask::pidParkAdjust = &pidParkAdjust;

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    CycleTimerInit();

    float sendData[4] = {0};
    uint32_t times = 0;

    while(1)
    {
        pidSpeed[0].SetPIDParam(&PIDMotorParam);
        pidSpeed[1].SetPIDParam(&PIDMotorParam);
        ESOSpeed[0].SetPIDParam(&ESOMotorParam);
        ESOSpeed[1].SetPIDParam(&ESOMotorParam);
        pidDistance[0].SetPIDParam(&PIDDistanceParam);
        pidDistance[1].SetPIDParam(&PIDDistanceParam);
        pidParkAdjust.SetPIDParam(&PIDParkAdjustParam);

        times++;

        car.StatusManage();
        car.ExecuteTask();
        car.PIDCompute();
//        car.OpenLoop(times*delay_s,ESOMotorParam.Kp);

//        sendData[0] = motors[0].omega_set;
//        sendData[1] = car.motor_output[0];
//        sendData[2] = motors[0].omega_actual;
//        sendData[3] = motors[1].omega_actual;

//        sendData[0] = carTask::parkParam.x;
//        sendData[1] = carTask::parkParam.y;
//        sendData[2] = carTask::parkParam.theta;
//        sendData[3] = carTask::parkParam.flag;


        sendData[0] = carTask::pathTrackTask.x_out[0];
        sendData[1] = carTask::pathTrackTask.x_out[1];
        sendData[2] = carTask::pathTrackTask.x_out[2];
        sendData[3] = carTask::pathTrackSim.carSim->phi *57.2957795;

//        sendData[0] = IMU_Angle[0];
//        sendData[1] = IMU_Angle[1];
//        sendData[2] = IMU_Angle[2];
//        sendData[3] = 0;

//        sendData[0] = 0;
//        sendData[0] = carTask::pathTrackSim.carSim->phi *57.2957795;
//        sendData[1] = IMU_Angle[2];
//        sendData[2] = 0;
//        sendData[3] = 0;

        //if(mqttConnected == true && times%6 == 0)
        if(times%2 == 0)
        {
            sendChartDataArray(times*delay_s,sendData);
        }
//        sendMatlabArray(times*delay_s,sendData);

        delayFlag = true;
        while(delayFlag);
    }
    return 0;
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
