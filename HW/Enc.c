#include "stm32f10x.h"
#include "cmtSys_TIM.h"

#define ENC_SPEED_SAMP_INTVL_MS  	10
#define ENC_CNT_ZERO_BASE   		32768 // 1<<15
#define ENC_LINES 					11
#define ENC_MULT					4	  // depends on the config of TIM Encoder Interface; typically `TIM_EncoderMode_TI12` ==> `4`
#define ENC_GEAR_RATIO  			9.6

/// @brief also serves as a state machine for speed sampling
typedef enum __ENC_SAMPLE_METHODS_DEF_ENUM {
	ENC_SAMP_METHOD_BY_ENC = 0,
	ENC_SAMP_METHOD_BY_IC
} EncSampMethodsDef_t;

/* Speed Measurement Related Macro*/
//tunables
#define ENC_LOW_SPEED_HANDOVER_THRESHOLD_RPM	20U		//!< RPM threshold for handing over control at low speeds.
#define ENC_HIGH_SPEED_HANDOVER_THRESHOLD_CPS	114514U	//!< CPS threshold for handing over control at high speeds
#define ENC_LOW_SPEED_HYST_BAND_RPM				10U		//!< Hysteresis for low-speed handover. @warning SHOULD be less than @ref ENC_LOW_SPEED_HANDOVER_THRESHOLD_RPM
#define ENC_HIGH_SPEED_HYST_BAND_CPS			1000U	//!< Hysteresis for high-speed handover
// #define ENC_LOW_SPEED_HYST_DEBOUNCE_TIMES	1U		//!< Delay for low-speed handover
// #define ENC_HIGH_SPEED_HYST_DEBOUNCE_TIMES	1U		//!< Delay for high-speed handover
#define ENC_IC_LAG_FILTER_COEF					0.78f

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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
						   RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 |
						   RCC_APB1Periph_TIM3, ENABLE);

	//GPIO - PA0--TIM2_CH1, PA1--TIM2_CH2; PA8--TIM1_CH1, PA9--TIM1_CH2; PA6--TIM3_CH1, PA7--TIM3_CH2
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);

	//TIM CLK
	TIM_InternalClockConfig(TIM1);
	TIM_InternalClockConfig(TIM2);

	//Timebase Unit for Encoder(TIM1,2)
	TIM_TimeBaseInitTypeDef timeBase;				
	timeBase.TIM_ClockDivision = TIM_CKD_DIV1;     
	timeBase.TIM_CounterMode = TIM_CounterMode_Up; 
	timeBase.TIM_Period = 65536 - 1;		// ARR
	timeBase.TIM_Prescaler = 1 - 1;			// PSC
	timeBase.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &timeBase);
	TIM_TimeBaseInit(TIM2, &timeBase);

	//Timebase Unit for IC(TIM3); MUST match PWM-Motor.c (its CH3/CH4 PWM shares this
	//counter) AND the `ENC_IC_CNT_MOD`/`ENC_IC_TICK_US` pair of this file
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 1000U - 1U;				//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72U - 1U;				//PSC
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

	//TIM IC for IC(TIM3); reuses Encoder's IC settings, both channels are pulse width inputs
	timIC.TIM_Channel = TIM_Channel_1;	//TIM3_CH1 --> L signal source
	TIM_ICInit(TIM3, &timIC);
	timIC.TIM_Channel = TIM_Channel_2;	//TIM3_CH2 --> R signal source
	TIM_ICInit(TIM3, &timIC);

	//Input TRIG
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Disable);	// MUST disable it or it will affect TIM3's CNT and disrupt PWM activities

	//TIM Encoder Interface
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	//TIM IC IT
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1 | TIM_IT_CC2);
	TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);	// Update IT is EN for detecting CNT wraps

	NVIC_InitTypeDef nvic;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);			// same grouping as cmtSys_TIM.c
	nvic.NVIC_IRQChannel = TIM3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0U;			// as urgent as the 1 ms system tick
	nvic.NVIC_IRQChannelSubPriority = 1U;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	//TIM EN
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);	// also enables the CH3/CH4 motor PWM of PWM-Motor.c
}

/* Speed Sampling */

static void Enc_SampleSpeedSign(uint8_t *speedSign)
{
	uint8_t sign = 0;
	uint8_t currCNT[2] = {0, 0};

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

/// @brief calculate a rough RPM from encoder counts
/// @param speedRPM float array to store the RPM values
/// @param nowMs current system time in Ms
/// @note - Assumption: CNT do not ovf or udf (ARR/2@32768); mind your `ENC_SPEED_SAMP_INTVL_MS`
/// @return - `1` if speed was updated
/// @return - `0` otherwise
static void Enc_SampleSpeedByEnc(float *speedRPM, float *speedCPS, uint32_t nowMs)
{
	static uint32_t s_nextSampMs  = 0;
	static uint8_t  s_firstSample = 1;

	uint16_t nowCNT[2]    = {0, 0};
	uint16_t deltaCNT[2]  = {0, 0};
	uint8_t  speedSign[2] = {0, 0};
	uint32_t elapsedMs;

	//get direction (speed sign)
	Enc_SampleSpeedSign(speedSign);

	if(s_firstSample)
	{// discard first sample (but still do the routine)
		Enc_GetCNT(nowCNT);	// also reset CNT
		s_firstSample = 0;
	}
	else
	{// normal routine
		Enc_GetCNT(nowCNT);	// also reset CNT
		elapsedMs = nowMs - s_nextSampMs;

		uint8_t i;
		for(i = 0; i < 2; i++)
		{
			if(speedSign[i] == 0)
			{//'+'
				deltaCNT[i] = nowCNT[i] - ENC_CNT_ZERO_BASE;
			}
			else
			{//'-'
				deltaCNT[i] = ENC_CNT_ZERO_BASE - nowCNT[i];
			}
		}

		speedRPM[0] = (speedSign[0] ? -1 : 1) *
					  (deltaCNT[0] /
					   ((ENC_LINES * ENC_GEAR_RATIO * ENC_MULT) * elapsedMs)
					  ) * 1000 * 60; // L speed

		speedRPM[1] = (speedSign[1] ? -1 : 1) *
					  (deltaCNT[1] /
					   ((ENC_LINES * ENC_GEAR_RATIO * ENC_MULT) * elapsedMs)
					  ) * 1000 * 60; // R speed

		speedCPS[0] = deltaCNT[0];
		speedCPS[1] = deltaCNT[1];

		s_nextSampMs = nowMs + ENC_SPEED_SAMP_INTVL_MS;
	}
}

/**
 * @param mode The sampling mode to specify use of this fn. 
 * 	This parameter can be one of these three values:
 * 	- @arg 0: Called by @ref Enc_ISR - sample one pulse. param `speedRPM` is not used
 * 	- @arg 1: Called by @ref Enc_SampleSpeed - update `speedRPM`. param `thisCNT` is not used
 */
static void Enc_SampleSpeedByIC(float *speedRPM, uint16_t thisCNT, uint8_t channel, uint8_t mode)
{
	static uint8_t  s_hasRisen[2] 	 = {0, 0}; // whether the channel already holds a valid previous capture
	static uint16_t s_lastCNT[2]  	 = {0, 0}; // last captured CNT
	static uint32_t s_lastMs[2]   	 = {0, 0}; // last captured time (low 16 bit of the ms tick suffices for `ENC_SPEED_SAMP_INTVL_MS`)
	static uint16_t s_filteredRPM[2] = {0, 0};
	static uint8_t  s_firstSample[2] = {1, 1};

	/* Body */

	uint32_t thisMs;
	uint32_t deltaTimeUs;
	uint8_t  speedSign[2];
	float    thisRPM;

	if(mode == 0)
	{
		Enc_SampleSpeedSign(speedSign);

		thisMs = CmtSysTim_GetMs();

		deltaTimeUs = (thisMs - s_lastMs[channel]) * 1000U +
					(thisCNT - s_lastCNT[channel]);

		s_lastCNT[channel] = thisCNT;
		s_lastMs[channel]  = thisMs;

		if((s_hasRisen[channel] == 0U) || (deltaTimeUs == 0U))
		{// no valid interval yet: only (re)arm the reference and drop this biased sample
			s_hasRisen[channel] = 1U;
			s_lastCNT[channel]  = thisCNT;
			s_lastMs[channel]   = thisMs;
			return;
		}

		thisRPM = (60 *1000000) /
				((ENC_LINES * ENC_GEAR_RATIO) * deltaTimeUs);

		//decides sign
		thisRPM *= (speedSign[channel] ? -1 : 1);

		if(s_firstSample[channel] == 1)
		{// init
			s_filteredRPM[channel] = thisRPM;
			return;
		}

		//lag filter
		s_filteredRPM[channel] += ENC_IC_LAG_FILTER_COEF * (thisRPM - s_filteredRPM[channel]);
	}
	else if(mode == 1)
	{
		speedRPM[0] = s_filteredRPM[0];
		speedRPM[1] = s_filteredRPM[1];
	}
}

/// @brief Get actual motor speed in RPM
/// @details based on the last sample, this fn routes to one of two sampling methods to calculate a more accurate RPM : 
/// @details - using Encoder Interface and pulse freq when the freq is high
/// @details - using IC and pulse width when the freq is low
/// @param speedRPM float array to store the RPM values
/// @param nowMs current system time in Ms
/// @note - Assumption: CNT do not ovf or udf (ARR/2@32768); mind your `ENC_SPEED_SAMP_INTVL_MS`
/// @return - `1` if speed was updated
/// @return - `0` otherwise
uint8_t Enc_SampleSpeed(float *speedRPM, uint32_t nowMs)
{
	static uint32_t s_nextSampMs = 0;
	static float s_lastRPM[2] = {0, 0};
	static float s_lastCPS[2] = {0, 0};
	static uint8_t  s_lastMethod = ENC_SAMP_METHOD_BY_ENC;
	//static uint8_t  s_hystMs     = 0;

	if(nowMs >= s_nextSampMs)
	{// Time to sample
		switch (s_lastMethod)
		{
		case ENC_SAMP_METHOD_BY_ENC:
			float sigLowRPM  = ENC_LOW_SPEED_HANDOVER_THRESHOLD_RPM + ENC_LOW_SPEED_HYST_BAND_RPM;
			float sigHighCPS = ENC_HIGH_SPEED_HANDOVER_THRESHOLD_CPS - ENC_HIGH_SPEED_HYST_BAND_CPS;
			// Check if we need to switch to the IC method
			if(((s_lastRPM[0] > sigLowRPM) && (s_lastRPM[0] < sigHighCPS)) ||
			   ((s_lastRPM[1] > sigLowRPM) && (s_lastRPM[1] < sigHighCPS)))
			{// conditions met; switch to IC method
				Enc_SampleSpeedByIC(speedRPM, (uint16_t)0, nowMs, 1);
				s_lastMethod = ENC_SAMP_METHOD_BY_IC;
			}
			else
			{// unchanged
				Enc_SampleSpeedByEnc(speedRPM, s_lastCPS, nowMs);
			}
			break;

		case ENC_SAMP_METHOD_BY_IC:
			float sigLowRPM  = ENC_LOW_SPEED_HANDOVER_THRESHOLD_RPM - ENC_LOW_SPEED_HYST_BAND_RPM;
			float sigHighCPS = ENC_HIGH_SPEED_HANDOVER_THRESHOLD_CPS + ENC_HIGH_SPEED_HYST_BAND_CPS;
			// Check if we need to switch to the Encoder method
			if(((s_lastCPS[0] < sigLowRPM) && (s_lastCPS[0] > sigHighCPS)) &&
			   ((s_lastCPS[1] < sigLowRPM) && (s_lastCPS[1] > sigHighCPS)))
			{// conditions met; switch to Encoder method
				Enc_SampleSpeedByEnc(speedRPM, s_lastCPS, nowMs);
				s_lastMethod = ENC_SAMP_METHOD_BY_ENC;
			}
			else
			{// unchanged
				Enc_SampleSpeedByIC(speedRPM, (uint16_t)0, nowMs, 1);
			}
			break;
		default:
			break;
		}

		s_nextSampMs = nowMs + ENC_SPEED_SAMP_INTVL_MS;
		return 1;
	}
	else// not yet...
		return 0;
}

void Enc_ISR(void)
{
	uint16_t captured[2] = {0, 0};
	uint8_t  needUpdate[2] = {0, 0};

    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {// L side sent the pulse
		captured[0] = TIM_GetCapture1(TIM3);	// NOTE that reading CCR clears the IT flag so we don't need to clear it manually
		needUpdate[0] = 1;
    }

	if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
	{// R side sent the pulse
		captured[1] = TIM_GetCapture2(TIM3);	// NOTE that reading CCR clears the IT flag so we don't need to clear it manually
		needUpdate[1] = 1;
	}

	Enc_SampleSpeedByIC((float[]){0.0f, 0.0f}, captured, 0, 0);
	Enc_SampleSpeedByIC((float[]){0.0f, 0.0f}, captured, 1, 0);
}
