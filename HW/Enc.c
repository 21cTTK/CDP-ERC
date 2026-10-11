#include "stm32f10x.h"
#include "cmtSys_TIM.h"
#include "Enc.h"

//Enc algo Tunables
#define ENC_CNT_ZERO_BASE   		32768 	//!< base number that serves as a '0' used in @ref Enc_SampleSpeedByEnc ; value 1<<15
#define ENC_IC_LAG_FILTER_COEF		0.78f	//!< lag filter coefficient used in @ref Enc_SampleSpeedByIC

/* Encoder properties Macro */
//tunables
#define ENC_SPEED_SAMP_INTVL_MS  	10
#define ENC_LINES 					11
#define ENC_MULT					4	  // depends on the config of TIM Encoder Interface; typically `TIM_EncoderMode_TI12` ==> `4`
#define ENC_GEAR_RATIO  			9.6
//derivative constants
#define ENC_COUNTS_PER_REV			(ENC_LINES * ENC_GEAR_RATIO * ENC_MULT)	//!< encoder counts (CNT ticks) per output-shaft rev for speed calculation using Enc (ref to @ref Enc_SampleSpeedByEnc )

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
	//Timebase Unit for IC(TIM3); MUST match PWM-Motor.c (its CH3/CH4 PWM shares this
	//counter) AND the `ENC_IC_PWM_ARR`/`ENC_IC_TICK_US` pair of this file
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = ENC_IC_PWM_ARR - 1U;		//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = ENC_IC_PSC_DIV - 1U;	//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0U;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

	//TIM IC for Encoder(TIM1,2)
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
	TIM_Cmd(TIM3, ENABLE);	// also enables the CH3/CH4 motor PWM of PWM-Motor.c
}

/* Speed Sampling */

static void Enc_SampleSpeedSign(uint8_t *speedSign)
{
	uint16_t currCNT[2] = {0, 0};

	currCNT[0] = TIM_GetCounter(TIM1);
	currCNT[1] = TIM_GetCounter(TIM2);

	uint8_t i;
	for(i = 0; i < 2; i++)
	{
		if(currCNT[i] >= ENC_CNT_ZERO_BASE)
		{
			speedSign[i] = 0;
		}
		else
		{
			speedSign[i] = 1;
		}
	}
}

/// @brief sample both channels with the Encoder Interface method
/// @param speedRPM float array to store the RPM values (always written)
/// @param speedCPS float array to store the raw CPS values (always written)
/// @param nowMs current system time in Ms
/// @param wen Write ENable flag. only write to `speedRPM` and `speedCPS` if this is set. used to init the method at the first samp of @ref Enc_SampleSpeed
/// @note - The whole routine is scheduled by `s_nextSampMs`; a late first call just yields a
///         longer first interval instead of leaving the caller's buffers untouched
/// @note - Assumption: CNT do not ovf or udf (ARR/2@32768); mind your `ENC_SPEED_SAMP_INTVL_MS`
static void Enc_SampleSpeedByEnc(float *speedRPM, uint16_t *speedCPS, uint32_t nowMs, uint8_t wen)
{
	static uint32_t s_nextSampMs  = 0;

	uint16_t thisCNT[2]   = {0, 0};
	uint16_t deltaCNT[2]  = {0, 0};
	uint8_t  speedSign[2] = {0, 0};
	uint32_t elapsedMs;

	//get direction (speed sign); MUST run before `Enc_GetCNT` resets the CNT
	Enc_SampleSpeedSign(speedSign);
	Enc_GetCNT(thisCNT);		// also reset CNT

	elapsedMs = nowMs - s_nextSampMs + ENC_SPEED_SAMP_INTVL_MS;

	s_nextSampMs = nowMs + ENC_SPEED_SAMP_INTVL_MS;

	if(wen == 1U)
	{
		uint8_t i;
		for(i = 0; i < 2; i++)
		{
			if(speedSign[i] == 0)
			{//'+'
				deltaCNT[i] = thisCNT[i] - ENC_CNT_ZERO_BASE;
			}
			else
			{//'-'
				deltaCNT[i] = ENC_CNT_ZERO_BASE - thisCNT[i];
			}
		}

		speedRPM[0] = (speedSign[0] ? -1.0f : 1.0f) *
					((float)deltaCNT[0] /
					 ((float)ENC_COUNTS_PER_REV * (float)elapsedMs)
					) * 1000.0f * 60.0f; // L speed

		speedRPM[1] = (speedSign[1] ? -1.0f : 1.0f) *
					((float)deltaCNT[1] /
					 ((float)ENC_COUNTS_PER_REV * (float)elapsedMs)
					) * 1000.0f * 60.0f; // R speed

		speedCPS[0] = (speedSign[0] ? -1 : 1) * deltaCNT[0];
		speedCPS[1] = (speedSign[1] ? -1 : 1) * deltaCNT[1];
	}
	else;
}

/**
 * @brief Get actual motor speed in RPM
 * @details routes per sampled result: the Encoder Interface method (pulse frequency) below
 * 		@ref ENC_LOW_SPEED_HANDOVER_THRESHOLD_RPM, the IC method (pulse width) above it,
 * 		so that the whole expected range (up to 620 RPM) is measured by the IC method
 * @param speedRPM float array to store the RPM values
 * @param nowMs current system time in Ms
 * @note - The two methods run in parallel and we ONLY CHOOSE to "trust" one of them
 * @note - Assumption: CNT do not ovf or udf (ARR/2@32768); mind your `ENC_SPEED_SAMP_INTVL_MS`
 * @return - `1` if speed was updated
 * @return - `0` otherwise
 */
uint8_t Enc_SampleSpeed(float *speedRPM, uint32_t nowMs)
{
	static uint8_t  s_firstSamp   = 1U;
	static uint32_t s_nextSampMs  = 0;
	// static uint8_t  s_lastMethod  = ENC_SAMP_METHOD_BY_ENC;

	if(s_firstSamp == 1U)
	{
		s_firstSamp = 0U;
		float *dummy;
		Enc_SampleSpeedByEnc(dummy, dummy, nowMs, 0U);
		return 1U;
	}

	if(nowMs >= s_nextSampMs)
	{// Time to sample
		float    encRPM[2];
		uint16_t encCPS[2];

		Enc_SampleSpeedByEnc(encRPM, encCPS, nowMs, 1U);

		/* Routing method */
		if(1)
		{
			speedRPM[0] = encRPM[0];
			speedRPM[1] = encRPM[1];
		}

		s_nextSampMs = nowMs + ENC_SPEED_SAMP_INTVL_MS;
		return 1;
	}
	else
		return 0;
}
