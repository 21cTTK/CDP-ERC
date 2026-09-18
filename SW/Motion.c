#include "stm32f10x.h"
#include "PWM-Motor.h"
#include "Motion.h"

/// @brief motion cfg tab for the robot;
/// @note used exclusively in `Motion_SetMotion()` to fill the input arr
static const uint8_t g_MotionCFG[MOTION_COUNT][MOTOR_PIN_COUNT] =
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
void Motion_SetMotion(Motion_t motion, uint16_t *argv)
{
    /* Set Input */
    Motor_SetInput(g_MotionCFG[(uint8_t)motion]);

    /* Set Speed */

    switch(motion)
    {
	case MOTION_STOP:
		Motor_L_SetSpeed(0.0f);
		Motor_R_SetSpeed(0.0f);
		break;
	case MOTION_FORWARD:
		Motor_L_SetSpeed(argv[0]);
		Motor_R_SetSpeed(argv[0]);
		break;
	case MOTION_BACKWARD:
		Motor_L_SetSpeed(argv[0]);
		Motor_R_SetSpeed(argv[0]);
		break;
	case MOTION_LEFT:
		Motor_L_SetSpeed(argv[0]);
		Motor_R_SetSpeed(argv[1]);
		break;
	case MOTION_RIGHT:
		Motor_L_SetSpeed(argv[0]);
		Motor_R_SetSpeed(argv[1]);
		break;
	default:
		break;
    }
}
