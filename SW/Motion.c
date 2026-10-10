#include "stm32f10x.h"
#include "PWM-Motor.h"
#include "Motion.h"

static uint8_t isStop;

/// @brief initialize the motion control (and motor)
void Motion_Init(void)
{
    Motor_Init();
}

void Motion_SetStop(uint8_t stopState)
{
    uint8_t dirInput[MOTOR_PIN_COUNT] = {0, 0, 0, 0};
    Motor_SetInput(dirInput);

    isStop = stopState;
}

uint8_t Motion_IsStopped(void)
{
    return isStop;
}

void Motion_SetSpeed(float *argv)
{
    uint8_t dirInput[MOTOR_PIN_COUNT] = {0, 0, 0, 0};

    if(argv[0] > 0.0f)
    { // L forward
        dirInput[0] = 0;
        dirInput[1] = 1;
        Motor_L_SetSpeed(argv[0]);
    }
    else
    { // L backward
        dirInput[0] = 1;
        dirInput[1] = 0;
        Motor_L_SetSpeed(-argv[0]);
    }

    if(argv[1] > 0.0f)
    { // R forward
        dirInput[2] = 0;
        dirInput[3] = 1;
        Motor_R_SetSpeed(argv[1]);
    }
    else
    { // R backward
        dirInput[2] = 1;
        dirInput[3] = 0;
        Motor_R_SetSpeed(-argv[1]);
    }
    Motor_SetInput(dirInput);
}
