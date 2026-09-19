#include "stm32f10x.h"                  // Device header
#include "PWM-Motor.h"
#include "Motion.h"
#include "Delay.h"
#include "cmtSys_TIM.h"
#include "LEDs.h"

void CmtSys_Init()
{
    /* Module Init */
    Motion_Init();
    LED_Init();

    // Init Indicator
    for(int i = 0; i < 5; i++)
    {
        LED_TurnON((LEDslot)LED_MAIN);
        Delay_ms(114);
        LED_TurnOFF((LEDslot)LED_MAIN);
        Delay_ms(114);
    }

    CmtSysTim_Init();

    /* Cleanup for startup */
    LED_SetFlicker(0, (LEDslot)LED_MSG, 5, 114);
}

void CmtSys_Update()
{
    uint32_t nowMs = CmtSysTim_GetMs();

    /* `Update()` Section */

    LED_CheckFlicker(nowMs);

    // float speed = 67.91;
	
	// if(nowMs < 1860)
	// 	Motion_SetMotion(MOTION_FORWARD, &speed);
	// else
	// 	Motion_SetMotion(MOTION_BACKWARD, &speed);
}