#include "stm32f10x.h"
#include "cmtSys_TIM.h"

#define ENC_SPEED_SAMP_INTVL_MS  	10
#define ENC_CNT_ZERO_BASE   		32768 // 1<<15
#define ENC_LINES 					11
#define ENC_MULT					4	  // depends on the config of TIM Encoder Interface; typically `TIM_EncoderMode_TI12` ==> `4`
#define ENC_GEAR_RATIO  			9.6

volatile float g_watch_RPM;

/* Speed Measurement Related Macro*/
//tunables
//#define ENC_TIM_ARR							1000U
//#define ENC_MIN_SAFE_IC_PULSE_WIDTH_CNT		50U		// Minimum safe TIM IC pulse width that doesn't drop pulses (deemed SAFE)
// #define ENC_SAMPLE_SWITCH_THRESHOLD_CPS		100U	//!< Unit is Count Per Sample
// //constants
// #define ENC_IC_PULSE_WIDTH_THRESHOLD_CNT	100U

static float    g_avgRPM[2] = {0, 0}; // average RPM over this turn
static uint16_t g_sampCnt[2] = {0, 0}; // sample count

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

static void Enc_SampleSpeedByIC(uint8_t channel)
{
	static uint8_t  s_hasRisen[2] = {0, 0}; // whether the channel already holds a valid previous capture
	static uint16_t s_lastCNT[2]  = {0, 0}; // last captured CNT
	static uint16_t s_lastMs[2]   = {0, 0}; // last captured time (low 16 bit of the ms tick suffices for `ENC_SPEED_SAMP_INTVL_MS`)

	/* Body */
	uint16_t thisCNT;
	uint16_t thisMs;
	uint32_t deltaTimeUs;
	float    thisRPM;

	//CCR1 carries the TIM3_CH1(L) capture, CCR2 carries the TIM3_CH2(R) one
	thisCNT = (channel == 0U) ? TIM_GetCapture1(TIM3) : TIM_GetCapture2(TIM3);
	thisMs  = (uint16_t)CmtSysTim_GetMs();

	// the ms diff MUST be taken in 16 bit modular arith first: promoting it to `int`
	// would turn a wrap of `thisMs` (< `s_lastMs`) into a negative interval
	deltaTimeUs = (uint32_t)(uint16_t)(thisMs - s_lastMs[channel]) * 1000U +
				  (uint16_t)(thisCNT - s_lastCNT[channel]);

	if((s_hasRisen[channel] == 0U) || (deltaTimeUs == 0U))
	{// no valid interval yet: only (re)arm the reference and drop this biased sample
		s_hasRisen[channel] = 1U;
		s_lastCNT[channel]  = thisCNT;
		s_lastMs[channel]   = thisMs;
		return;
	}

	thisRPM = (60 *1000000) /
			  ((ENC_LINES * ENC_GEAR_RATIO * (ENC_MULT / 2) ) * deltaTimeUs);

	g_avgRPM[channel] = (g_avgRPM[channel] * g_sampCnt[channel] + thisRPM) / (++g_sampCnt[channel]);

	s_lastCNT[channel] = thisCNT;
	s_lastMs[channel]  = thisMs;
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
	static uint32_t s_nextSampMs  = 0;
	static uint8_t  s_firstSample = 1;

	if(s_firstSample)
	{// discard first sample (but still do the routine)
		s_nextSampMs = nowMs + ENC_SPEED_SAMP_INTVL_MS;
		s_firstSample = 0;
		return 0;
	}
	else if(nowMs >= s_nextSampMs)
	{
		// snapshot + clear MUST NOT interleave with Enc_ISR's read-modify-write of
		// `g_avgRPM`/`g_sampCnt`, or a stale sample leaks into the next window
		TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2, DISABLE);

		speedRPM[0] = g_avgRPM[0];
		speedRPM[1] = g_avgRPM[1];

		g_watch_RPM = speedRPM[0];

		g_avgRPM[0] = 0;
		g_avgRPM[1] = 0;
		g_sampCnt[0] = 0;
		g_sampCnt[1] = 0;

		TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

		s_nextSampMs = nowMs + ENC_SPEED_SAMP_INTVL_MS;
		return 1;
	}
	else
		return 0;
}

void Enc_ISR(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {
      TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
      Enc_SampleSpeedByIC(0);
    }

	if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
	{
	  TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	  Enc_SampleSpeedByIC(1);
	}
}
