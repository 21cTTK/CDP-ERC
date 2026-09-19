#include "stm32f10x.h"
#include "Keys.h"

static uint32_t g_SysMs;

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
	Key_CheckState_SysTick();
}

uint32_t CmtSysTim_GetMs(void)
{
	return g_SysMs;
}
