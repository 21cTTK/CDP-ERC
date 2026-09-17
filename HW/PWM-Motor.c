#include "stm32f10x.h"                  // Device header

//static uint8_t g_MotorIsON_L;
//static uint8_t g_MotorIsON_R;

void Motor_Init(void)
{
	/* PWM CFG Section */

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |
						   RCC_APB2Periph_GPIOA, ENABLE);
	
	// PWM-GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	// TIM3_CH1_4 --> PA6,PA7,PB0,PB1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// TIM3 - Time Base Unit Init
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 1000U - 1U;				//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72U - 1U;				//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0U;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	// TIM3_OC1_4 Init
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0U;								//CCR initial value

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	// TIM3 EN
	TIM_Cmd(TIM3, ENABLE);

	/* Driver pins Section */

	// Driver AIN EN (PA3_0)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);	// Brake
}

/// @brief set speed of the motor by entering percentage
/// @param perc float val like 11.45, ...
void Motor_SetSpeed_M0(float perc)
{// actual_percentage(duty) = (CCR / 1000) * 100%
	TIM_SetCompare4(TIM3, (uint16_t)(perc * 10.0f));
}

void Motor_SetSpeed_M1(float perc)
{
	TIM_SetCompare3(TIM3, (uint16_t)(perc * 10.0f));
}

void Motor_SetSpeed_M2(float perc)
{// actual_percentage(duty) = (CCR / 1000) * 100%
	TIM_SetCompare2(TIM3, (uint16_t)(perc * 10.0f));
}

void Motor_SetSpeed_M3(float perc)
{
	TIM_SetCompare1(TIM3, (uint16_t)(perc * 10.0f));
}

/// @brief switch motor ON/OFF
/// @param motor can be `0`(motor_L) or `1`(motor_R)
void Motor_SwitchState(uint8_t motor)
{
	if (motor == 0) {
		// Switch ON motor_L
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
	} else {
		// Switch ON motor_R
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	}
}
