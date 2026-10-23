#include "stm32f10x.h"
#include "Keys.h"
#include "LEDs.h"
#include "Motion.h"
#include "PID.h"
#include "Enc.h"
#include "MPU6050.h"
#include <math.h>

static uint32_t g_SysMs;
volatile float tg_offsetRPM;
volatile float etg_actualSet;

void CmtSysTim_Init(void)
{
	TIM_TimeBaseInitTypeDef timeBase;
	NVIC_InitTypeDef nvic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	timeBase.TIM_Prescaler = 72U - 1U;		//PSC
	timeBase.TIM_Period = 1000U - 1U;		//ARR
	timeBase.TIM_CounterMode = TIM_CounterMode_Up;
	timeBase.TIM_ClockDivision = TIM_CKD_DIV1;
	timeBase.TIM_RepetitionCounter = 0U;
	TIM_TimeBaseInit(TIM4, &timeBase);

	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	nvic.NVIC_IRQChannel = TIM4_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0U;
	nvic.NVIC_IRQChannelSubPriority = 0U;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	g_SysMs = 0U;
	TIM_Cmd(TIM4, ENABLE);
}

void CmtSysTim_ISR(void)
{
	g_SysMs++;

	/* Prioritized Activities */
	Key_CheckState_SysTick();			// uses inner counter
	LED_CheckFlicker_SysTick(g_SysMs);	// depends on `g_SysMs`

	if(Enc_SampleSpeed_SysTick(g_SysMs) != 0)
    {// yes we have a newly sampled speed; now up to PID
		PidParams_t* speedPID[2];
		PidParams_t* offsetPID;
		uint8_t      shouldStop;
		float        savedRampPointL;
		float		 savedRampPointR;
		float 		 controlledPWM[2];

		Enc_GetSpeedPidParams(0, &speedPID[0]);
		Enc_GetSpeedPidParams(1, &speedPID[1]);
		MPU6050_GetOffsetPidParams(&offsetPID);

		PID_Ramp(speedPID[0]);
		PID_Ramp(speedPID[1]);

		savedRampPointL = speedPID[0]->rampPoint;
		savedRampPointR = speedPID[1]->rampPoint;

		// can't rotate in reverse
		speedPID[0]->rampPoint -= offsetPID->output;
		speedPID[1]->rampPoint += offsetPID->output;
		etg_actualSet = speedPID[0]->rampPoint;

		shouldStop = Motion_IsStopped();
		PID_Update(speedPID[0], shouldStop);
		PID_Update(speedPID[1], shouldStop);

		// prevent accumulation of the trim: Update with trimmed value and reset to original value
		if(shouldStop == 0U)
		{// undo a live trim
			speedPID[0]->rampPoint = savedRampPointL;
			speedPID[1]->rampPoint = savedRampPointR;
		}

		controlledPWM[0] = speedPID[0]->output;
		controlledPWM[1] = speedPID[1]->output;

        Motion_SetSpeed(controlledPWM);
    }

    if((MPU6050_IsReady() != 0U) &&
	   (MPU6050_SampleTimeReached(g_SysMs)) != 0U)
    {// yep it can work and time to sample it
		// MPU6050's PID produces an offset speed on the left wheel to compensate for yaw
		// which stacks on Enc's PID
		PidParams_t* speedPID;
		PidParams_t* offsetPID;
		uint8_t shouldStop;
		int16_t  yawRaw;	//!< Yaw data.

		if(MPU6050_ReadYawData(&yawRaw) == 0U)
		{// yes! we got it
			MPU6050_YawRawToDps(yawRaw);

			MPU6050_GetOffsetPidParams(&offsetPID);
			Enc_GetSpeedPidParams(0, &speedPID);

			PID_Ramp(offsetPID);

			shouldStop = (Motion_IsStopped() || speedPID->setPoint == 0.0f);
			PID_Update(offsetPID, shouldStop);
			tg_offsetRPM = offsetPID->output;	// for debug			
		}
    }
}

uint32_t CmtSysTim_GetMs(void)
{
	return g_SysMs;
}
