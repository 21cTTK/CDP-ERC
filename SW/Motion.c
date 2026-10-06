#include "stm32f10x.h"
#include "PWM-Motor.h"
#include "Motion.h"

/// @brief motion cfg tab for the robot;
/// @note used exclusively in `Motion_SetMotion()` to fill the input arr
static const uint8_t g_MotionPinsCFG[MOTION_COUNT][MOTOR_PIN_COUNT] =
{
    [MOTION_STOP]     = {0, 0, 0, 0},
    [MOTION_FORWARD]  = {0, 1, 0, 1},
    [MOTION_BACKWARD] = {1, 0, 1, 0},
    [MOTION_LEFT]     = {0, 1, 1, 0},
    [MOTION_RIGHT]    = {1, 0, 0, 1},
};

/// @brief initialize the motion control (and motor)
void Motion_Init(void)
{
    Motor_Init();
}

/// @brief set the motion of the robot
/// @param motion the motion to set; can be any of the values in Motion_t
/// @param argv the speed percentage arguments for the motion; its meaning varies across different motions
/// @param targetRPM the target RPM for the motion
void Motion_SetMotion(Motion_t motion, float *argv, float *targetRPM)
{
    /* Set Input */
    Motor_SetInput(g_MotionPinsCFG[(uint8_t)motion]);

    /* Record target RPM */
	targetRPM[0] = argv[0];
	targetRPM[1] = argv[1];
}

void Motion_SetSpeed(float *argv)
{
    /* Set Speed */
    Motor_L_SetSpeed(argv[0]);
    Motor_R_SetSpeed(argv[1]);
}
