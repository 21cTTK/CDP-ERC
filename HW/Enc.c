#include "stm32f10x.h"

#define ENC_SPEED_SAMP_INTVL_MS  	520
#define ENC_CNT_ZERO_BASE   32768 // 1<<15
#define ENC_LINES 			11
#define ENC_MULT			4	  // depends on the config of TIM Encoder Interface; typically `TIM_EncoderMode_TI12` ==> `4`
#define ENC_GEAR_RATIO  	9.6

/// @brief Read in curr CNT and reset it to `ENC_CNT_ZERO_BASE`.
/// @note - real CNT ticks are calculated based on delta with `ENC_CNT_ZERO_BASE` to prevent ovf/udf
/// @note - used exclusively in @ref `uint8_t Enc_SampleSpeed(float *speedRPM, uint32_t nowMs)`
/// @param cnt value storage array
static void Enc_GetCNT(uint16_t *cnt)
{
	cnt[0] = TIM_GetCounter(TIM1);
	cnt[1] = TIM_GetCounter(TIM2);
	TIM_SetCounter(TIM1, ENC_CNT_ZERO_BASE);
	TIM_SetCounter(TIM2, ENC_CNT_ZERO_BASE);
}

void Enc_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|
						   RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	//GPIO - PA0--TIM2_CH1, PA1--TIM2_CH2; PA8--TIM1_CH1, PA9--TIM1_CH2
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);

	//TIM CLK
	TIM_InternalClockConfig(TIM1);
	TIM_InternalClockConfig(TIM2);

	//Timebase Unit
	TIM_TimeBaseInitTypeDef timeBase;				
	timeBase.TIM_ClockDivision = TIM_CKD_DIV1;     
	timeBase.TIM_CounterMode = TIM_CounterMode_Up; 
	timeBase.TIM_Period = 65536 - 1;		// ARR
	timeBase.TIM_Prescaler = 1 - 1;			// PSC
	timeBase.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &timeBase);
	TIM_TimeBaseInit(TIM2, &timeBase);

	//TIM IC
	TIM_ICInitTypeDef timIC;
	TIM_ICStructInit(&timIC); // fill with default values
	timIC.TIM_Channel = TIM_Channel_1;
	timIC.TIM_ICFilter = 0xF;
	TIM_ICInit(TIM1, &timIC);
	TIM_ICInit(TIM2, &timIC);
	timIC.TIM_Channel = TIM_Channel_2;
	timIC.TIM_ICFilter = 0xF;
	TIM_ICInit(TIM1, &timIC);
	TIM_ICInit(TIM2, &timIC);

	//TIM Encoder Interface
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	//TIM EN
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

/// @brief calculate a rough RPM from encoder counts
/// @param speedRPM float array to store the RPM values
/// @param nowMs current system time in Ms
/// @note - Assumption: CNT do not ovf or udf (ARR/2@32768); mind your `ENC_SPEED_SAMP_INTVL_MS`
/// @return - `1` if speed was updated
/// @return - `0` otherwise
uint8_t Enc_SampleSpeed(float *speedRPM, uint32_t nowMs)
{
	static uint32_t s_nextSampMs  = 0;

	uint16_t nowCNT[2]    = {0, 0};
	uint16_t deltaCNT[2]  = {0, 0};
	uint8_t  speedSign[2] = {0, 0};
	uint32_t elapsedMs = nowMs - s_nextSampMs + ENC_SPEED_SAMP_INTVL_MS;
	if(nowMs >= s_nextSampMs)
	{
		Enc_GetCNT(nowCNT);	// also reset CNT
		
		if(nowCNT[0] >= ENC_CNT_ZERO_BASE)
		{
			deltaCNT[0] = nowCNT[0] - ENC_CNT_ZERO_BASE;
			speedSign[0] = 0;
		}
		else
		{
			deltaCNT[0] = ENC_CNT_ZERO_BASE - nowCNT[0];
			speedSign[0] = 1;
		}//L CNT

		if(nowCNT[1] >= ENC_CNT_ZERO_BASE)
		{
			deltaCNT[1] = nowCNT[1] - ENC_CNT_ZERO_BASE;
			speedSign[1] = 0;
		}
		else
		{
			deltaCNT[1] = ENC_CNT_ZERO_BASE - nowCNT[1];
			speedSign[1] = 1;
		}//R CNT

		speedRPM[0] = (speedSign[0] ? -1 : 1) *
					  (deltaCNT[0] /
					   ((ENC_LINES * ENC_GEAR_RATIO * ENC_MULT) *
					    (elapsedMs + (deltaCNT[0] / 1000))
					   )
					  ) * 1000 * 60; // L speed

		speedRPM[1] = (speedSign[1] ? -1 : 1) *
					  (deltaCNT[1] /
					   ((ENC_LINES * ENC_GEAR_RATIO * ENC_MULT) *
					    (elapsedMs + (deltaCNT[1] / 1000))
					   )
					  ) * 1000 * 60; // R speed
	    
		s_nextSampMs = nowMs + ENC_SPEED_SAMP_INTVL_MS;
		return 1;
	}
	else
		return 0;
}
