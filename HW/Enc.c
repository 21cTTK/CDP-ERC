#include "stm32f10x.h"
#include "cmtSys_TIM.h"
#include "Enc.h"
#include <math.h>

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
#define ENC_PULSES_PER_REV			(ENC_LINES * ENC_GEAR_RATIO)			//!< pulses per output-shaft rev on ONE channel for speed calculation using Enc (ref to @ref Enc_SampleSpeedByIC )

/// @brief also serves as a state machine for speed sampling
typedef enum __ENC_SAMPLE_METHODS_DEF_ENUM {
	ENC_SAMP_METHOD_BY_ENC = 0,
	ENC_SAMP_METHOD_BY_IC
} EncSampMethodsDef_t;

/* Speed Measurement Macro */
//tunables
#define ENC_LOW_SPEED_HANDOVER_THRESHOLD_RPM	20.0f	//!< RPM threshold for handing over control at low speeds.
#define ENC_HIGH_SPEED_HANDOVER_THRESHOLD_CPS	114514U	//!< CPS threshold for handing over control at high speeds
#define ENC_LOW_SPEED_HYST_BAND_RPM				10.0f	//!< Hysteresis for low-speed handover. @warning SHOULD be less than @ref ENC_LOW_SPEED_HANDOVER_THRESHOLD_RPM
#define ENC_HIGH_SPEED_HYST_BAND_CPS			1000U	//!< Hysteresis for high-speed handover
#define ENC_IC_EDGE_TIMEOUT_MS					200U	//!< A channel with no captured edge for this long is reported as 0 RPM (guard against a stale IC reading)
// #define ENC_LOW_SPEED_HYST_DEBOUNCE_TIMES	1U		//!< Delay for low-speed handover
// #define ENC_HIGH_SPEED_HYST_DEBOUNCE_TIMES	1U		//!< Delay for high-speed handover

/**
 * @var g_WrapCount
 * @brief IC CNT Wrap Counter (shared by both channels).
 * @details count how many times the IC CNT has wrapped around and used to decide the CNT position
 * @note - Writes only in @ref Enc_ISR (plus init at @ref Enc_Init );
 * @note - Reads only in @ref Enc_GetICTimeUs .
 */
static volatile uint32_t g_WrapCount = 0;

/* TIM3 IC Sampling State */
static uint8_t  g_hasRisen[2]    = {0, 0};			// whether the channel already holds a valid previous edge
static uint32_t g_lastEdgeUs[2]  = {0, 0};			// last captured edge on the TIM3 us time base (wrap-extended)
static uint32_t g_lastEdgeMs[2]  = {0, 0};			// last captured edge on the ms time base (staleness check)
static float    g_filteredRPM[2] = {0.0f, 0.0f};	// lag-filtered IC RPM

/// @brief (re)arm the IC method
/// @details the next edge of each channel only becomes the reference, and a channel that stays
/// 		 silent for @ref ENC_IC_EDGE_TIMEOUT_MS is treated as stopped
/// @note - MUST be called on every hand-over to the IC method, or the held reading of a channel
///         that has not produced an edge since the last stop would be reported forever
static void Enc_ArmIC(void)
{
	uint8_t i;
	for(i = 0; i < 2; i++)
	{
		g_hasRisen[i]   = 0U;
		g_lastEdgeMs[i] = CmtSysTim_GetMs();
	}
}

/// @brief didn't actually use the captured CNT in CCR and read the actual CNT ourselves
/// @warning return type `uint32_t` only lasts for ~71min counting time in us.
static uint32_t Enc_GetICTimeUs(void)
{
	uint32_t wrap = 0;
	uint16_t cnt  = 0;

	do 
	{//read wrap & CNT coherently (classic retry if an UPDATE landed in between)
		wrap  = g_WrapCount;
		cnt   = TIM3->CNT;	// SHOULD directly read the reg or it MAY be stale by several ticks
	} while(wrap != g_WrapCount);

	return (wrap * (uint32_t)ENC_IC_WRAP_US) + ((uint32_t)cnt * (uint32_t)ENC_IC_TICK_US);
}

/// @brief Read in curr CNT and reset it to `ENC_CNT_ZERO_BASE`.
/// @note - real CNT ticks are calculated based on delta with `ENC_CNT_ZERO_BASE` to prevent ovf/udf
/// @note - used exclusively in @ref Enc_SampleSpeedByEnc
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

	//TIM IC for IC(TIM3); reuses Encoder's IC settings, both channels are pulse width inputs
	timIC.TIM_Channel = TIM_Channel_1;	//TIM3_CH1 --> L signal source
	TIM_ICInit(TIM3, &timIC);
	timIC.TIM_Channel = TIM_Channel_2;	//TIM3_CH2 --> R signal source
	TIM_ICInit(TIM3, &timIC);

	//Input TRIG
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Disable);	// MUST disable it or it will affect TIM3's CNT and disrupt PWM activities

	//TIM Encoder Interface
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	//TIM IC IT
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2);
	TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);	// edge captures; the Update IT (CNT wrap) is polled by `Enc_GetICTimeUs`, not served
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);				// only feeds the wrap counter - MUST be cleared first thing in `Enc_ISR()`

	NVIC_InitTypeDef nvic;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);			// same grouping as cmtSys_TIM.c
	nvic.NVIC_IRQChannel = TIM3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0U;			// as urgent as the 1 ms system tick
	nvic.NVIC_IRQChannelSubPriority = 1U;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	//CNT Init: 0 for TIM3 (an absolute 1 us time base for the IC method), mid-scale for TIM1/2
	TIM_SetCounter(TIM3, 0U);
	TIM_SetCounter(TIM1, ENC_CNT_ZERO_BASE);
	TIM_SetCounter(TIM2, ENC_CNT_ZERO_BASE);
	g_WrapCount = 0U;
	Enc_ArmIC();

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
/// @param wen Write ENable flag. only write to `speedRPM` and `speedCPS` if this is set
/// @note - The whole routine is scheduled by `s_nextSampMs`; a late first call just yields a
///         longer first interval instead of leaving the caller's buffers untouched
/// @note - Assumption: CNT do not ovf or udf (ARR/2@32768); mind your `ENC_SPEED_SAMP_INTVL_MS`
static void Enc_SampleSpeedByEnc(float *speedRPM, uint16_t *speedCPS, uint32_t nowMs)
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

	if(elapsedMs == 0U)
	{// nothing to measure (first sample / same-ms re-entry)
		speedRPM[0] = 0.0f;
		speedRPM[1] = 0.0f;
		speedCPS[0] = 0.0f;
		speedCPS[1] = 0.0f;
		return;
	}
	else
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

		speedRPM[0] = (speedSign[0] ? -1 : 1) *
					((float)deltaCNT[0] /
					 ((float)ENC_COUNTS_PER_REV * (float)elapsedMs)
					) * 1000.0f * 60.0f; // L speed

		speedRPM[1] = (speedSign[1] ? -1 : 1) *
					((float)deltaCNT[1] /
					 ((float)ENC_COUNTS_PER_REV * (float)elapsedMs)
					) * 1000.0f * 60.0f; // R speed

		speedCPS[0] = (speedSign[0] ? -1 : 1) * deltaCNT[0];
		speedCPS[1] = (speedSign[1] ? -1 : 1) * deltaCNT[1];
	}
}

/**
 * @param speedRPM float array to store the RPM values; only written in ` @p mode == 1`
 * @param channel which channel this call refers to (0: L, 1: R)
 * @param mode The sampling mode to specify use of this fn. 
 * 	This parameter can be one of these three values:
 * 	- @arg 0: Called by @ref Enc_ISR - sample one pulse. param `speedRPM` is not used
 * 	- @arg 1: Called by @ref Enc_SampleSpeed - update `speedRPM`. param `thisCNT` is not used
 *  - @arg 2: Called by @ref Enc_SampleSpeed - do nothing. serves the same purpose as `wen == 0` of @ref Enc_SampleSpeedByEnc
 */
static void Enc_SampleSpeedByIC(float *speedRPM, uint8_t channel, uint8_t mode)
{
	/* Body */

	uint32_t thisUs;
	uint32_t deltaTimeUs;
	uint8_t  speedSign[2];
	float    thisRPM;

	if(mode == 0U)
	{
		//direction; MUST run before the capture bookkeeping
		Enc_SampleSpeedSign(speedSign);

		thisUs = Enc_GetICTimeUs();

		//period of THIS edge against the previous one, both on the free-running us time base
		deltaTimeUs = (thisUs - g_lastEdgeUs[channel]) / ENC_IC_TICK_US;

		g_lastEdgeUs[channel] = thisUs;
		g_lastEdgeMs[channel] = CmtSysTim_GetMs();

		if(g_hasRisen[channel] == 0U)
		{// first edge since the method was armed: no interval to measure yet, just take the reference
			g_hasRisen[channel] = 1U;
			return;
		}

		//scq before computation
		if((deltaTimeUs == 0U) || (deltaTimeUs > ENC_IC_WRAP_US))
		{// considered invalid
			return;
		}

		//core calc
		thisRPM = (60.0f * 1000000.0f) / ((float)ENC_PULSES_PER_REV * (float)deltaTimeUs);

		//decides sign
		thisRPM *= (speedSign[channel] ? -1.0f : 1.0f);

		//lag filter
		if(g_filteredRPM[channel] == 0.0f)
		{// init (also re-seeds after a standstill(`g_filteredRPM[i]  = 0.0f;` below))
			g_filteredRPM[channel] = thisRPM;
		}
		else
		{// apply lag filter
			g_filteredRPM[channel] += ENC_IC_LAG_FILTER_COEF * (thisRPM - g_filteredRPM[channel]);
		}
	}
	else if(mode == 1U)
	{
		uint8_t i;
		for(i = 0; i < 2; i++)
		{
			if(((CmtSysTim_GetMs() - g_lastEdgeMs[i]) > ENC_IC_EDGE_TIMEOUT_MS) || (g_hasRisen[i] == 0U))
			{// no edge since the method was armed / for a long time
				g_filteredRPM[i]  = 0.0f;
				g_lastEdgeMs[i]   = CmtSysTim_GetMs();	// reset the last edge timestamp
				speedRPM[i]       = 0.0f;	// reports zero and handover to Enc method
				continue;
			}

			speedRPM[i] = g_filteredRPM[i];
		}
	}
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
	static uint32_t s_nextSampMs  = 0;
	static uint8_t  s_lastMethod  = ENC_SAMP_METHOD_BY_ENC;

	if(nowMs >= s_nextSampMs)
	{// Time to sample
		float encRPM[2];
		uint16_t encCPS[2];

		//always sample with the Encoder method first to get CPS readings
		Enc_SampleSpeedByEnc(encRPM, encCPS, nowMs);

		switch (s_lastMethod)
		{
		case ENC_SAMP_METHOD_BY_ENC:
			float sigLowRpmEnc  = ENC_LOW_SPEED_HANDOVER_THRESHOLD_RPM  + ENC_LOW_SPEED_HYST_BAND_RPM;
			float sigHighCpsEnc = ENC_HIGH_SPEED_HANDOVER_THRESHOLD_CPS - ENC_HIGH_SPEED_HYST_BAND_CPS;
			if(((fabsf(encRPM[0]) >= sigLowRpmEnc) &&
			    (fabsf(encCPS[0]) <= sigHighCpsEnc)) ||
			   ((fabsf(encRPM[1]) >= sigLowRpmEnc) &&
			    (fabsf(encCPS[1]) <= sigHighCpsEnc)))
			{// measured abs val below the top of LOW zone OR above the bottom of HIGH zone --> still with Enc
				speedRPM[0] = encRPM[0];
				speedRPM[1] = encRPM[1];
			}
			else
			{// measured abs val above the top of LOW zone AND below the bottom of HIGH zone --> hand over to IC
				Enc_ArmIC();	// the method needs its own reference edges before it can report
				Enc_SampleSpeedByIC(speedRPM, 0, 1);
				s_lastMethod = ENC_SAMP_METHOD_BY_IC;
			}
			break;

		case ENC_SAMP_METHOD_BY_IC:
			
			//always sample with the Encoder method first
			Enc_SampleSpeedByIC(speedRPM, 0, 1);
		}

		s_nextSampMs = nowMs + ENC_SPEED_SAMP_INTVL_MS;
		return 1;
	}
	else// not yet...
		return 0;
}

void Enc_ISR(void)
{
	//the CNT wrap IT MUST be served first: it feeds the time base of every capture below
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		g_WrapCount++;
	}

	if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{// L side sent the pulse
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		Enc_SampleSpeedByIC((float[]){0.0f, 0.0f}, 0, 0);
	}

	if(TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
	{// R side sent the pulse
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		Enc_SampleSpeedByIC((float[]){0.0f, 0.0f}, 1, 0U);
	}
}
