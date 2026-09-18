#include "stm32f10x.h"                  // Device header
#include "PWM-Motor.h"

static uint16_t Motor_L_Speed;
static uint16_t Motor_R_Speed;

/* Helper Functions */

// none as to now...

/* API Functions */

/// @brief only used in its wrapper `Motion_Init()`
void Motor_Init(void)
{
	/* PWM CFG Section */

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	// PWM-GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	// TIM3_CH3_4 --> PB0,PB1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// TIM3 - Time Base Unit Init
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 1000U - 1U;				//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72U - 1U;				//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0U;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	// TIM3_OC3_4 Init
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0U;								//CCR initial value

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	// TIM3 EN
	TIM_Cmd(TIM3, ENABLE);

	/* Driver pins Section */

	// Driver AIN/BIN EN (PB15_12)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	for(int i = 0; i < MOTOR_PIN_COUNT; i++)
	{
		GPIO_InitStructure.GPIO_Pin = g_MotorPinsDef[i].GPIO_Pin;
		GPIO_Init(g_MotorPinsDef[i].GPIOx, &GPIO_InitStructure);
		GPIO_ResetBits(g_MotorPinsDef[i].GPIOx, g_MotorPinsDef[i].GPIO_Pin); 	// Init Brake
	}
}

/// @brief helper fn to set speed of the motor by entering percentage
/// @param perc float val like 11.45, ...
void Motor_L_SetSpeed(float perc)
{// actual_percentage(duty) = (CCR / 1000) * 100%
	TIM_SetCompare3(TIM3, (uint16_t)(perc * 10.0f));
}

void Motor_R_SetSpeed(float perc)
{
	TIM_SetCompare4(TIM3, (uint16_t)(perc * 10.0f));
}

/// @brief set TB6612FNG AIN/BIN pins to control the direction of the motor
/// @param in an arr containing the input values for the motor pins; should be of size `MOTOR_PIN_COUNT`
void Motor_SetInput(const uint8_t in[MOTOR_PIN_COUNT])
{
	for (int i = 0; i < MOTOR_PIN_COUNT; i++)
	{
		if(in[i])
			GPIO_SetBits(g_MotorPinsDef[i].GPIOx, g_MotorPinsDef[i].GPIO_Pin);
		else
			GPIO_ResetBits(g_MotorPinsDef[i].GPIOx, g_MotorPinsDef[i].GPIO_Pin);
	}
}
