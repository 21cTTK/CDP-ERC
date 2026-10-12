#include "stm32f10x.h"
#include "cmtSys_TIM.h"
#include "Enc.h"

volatile float 	  tg_icRPM;
volatile float 	  tg_encRPM;

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
#define ENC_PULSES_PER_REV			(ENC_LINES * ENC_GEAR_RATIO)			//!< encoder pulses (PWM ticks) per output-shaft rev

/* Speed Calculation Macro */
#define ENC_IC_EDGE_TIMEOUT_US   	114514U
#define ENC_IC_MIN_PULSE_INTVL_US	25U

static volatile uint32_t g_WrapCount;	//!< Count how many counter wraps are there

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

/// @brief didn't actually use the captured CNT in CCR and read the actual CNT ourselves
/// @warning return type `uint32_t` only lasts for ~71min counting time in us.
static uint32_t Enc_GetIcTimeUs(void)
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

/**
 * @param speedRPM float array to store the RPM values; only written in ` @p wen == 1`
 * @param channel which channel this call refers to (0: L, 1: R)
 * @param mode The sampling mode to specify use of this fn. 
 * @param wen Write ENable flag. only write to `speedRPM` if this is set. otherwise just sample a pulse.
 */
static void Enc_SampleSpeedByIC(float *speedRPM, uint8_t channel, uint8_t wen)
{
	static uint32_t s_lastEdgeUs[2]  = {0U, 0U};
	static uint8_t  s_icArmed[2]     = {0U, 0U};	//!< whether the IC method is armed for each channel
	
	/**
	 * @var s_filteredRPM
	 * @brief filtered RPM for each channel.
	 * @note - Also serve as a validity check variable.
	 * 		reset-value `0.0f` is considered invalid while non-zero value is valid to be read.
	 * @note - `volatile`-ize it because it may be changed by ISR mid-read
	 * 		(maybe validate while called with `wen` == 1 and change the output value)
	 */
	static volatile float s_filteredRPM[2] = {0U, 0U};

	/* Body */

	uint32_t currEdgeUs;
	uint32_t deltaTimeUs;
	uint8_t  speedSign[2];
	float    currRPM;

	if(wen == 0U) // just sample a pulse. "output" `speedRPM` is not modified in this mode
	{
		//direction; MUST run before the capture bookkeeping
		Enc_SampleSpeedSign(speedSign);

		currEdgeUs = Enc_GetIcTimeUs();

		//period of curr edge against the previous one, both on the free-running us time base
		deltaTimeUs = (currEdgeUs - s_lastEdgeUs[channel]) / ENC_IC_TICK_US;

		// scq-1: Min Pulse Interval Check
		if(deltaTimeUs <= ENC_IC_MIN_PULSE_INTVL_US)
		{// too short since the last edge; considered the same edge as before
			// `s_icArmed[channel]` is not updated. It makes sense
			// `s_lastEdgeUs[channel]` is not updated, stacking up until we pass this check
			// `s_filteredRPM[channel]` is not updated either since it's considered the same edge
			return;
		}

		// safe to save a record
		s_lastEdgeUs[channel] = currEdgeUs;

		// scq-2: Max Pulse Interval Check
		if(deltaTimeUs > ENC_IC_EDGE_TIMEOUT_US)
		{// too long since the last edge; considered a standstill and this maybe just a noise
			s_icArmed[channel] = 0U;		//re-seed a ref edge next time to confirm the activity
			s_filteredRPM[channel] = 0.0f;	//reset `s_filteredRPM` (invalidate it)
			return;
		}

		// Armitivity check before computation
		if(s_icArmed[channel] == 0U)
		{// hasn't seen a valid reference edge yet; take curr edge as the valid reference.
			s_icArmed[channel] = 1U;		//arm this channel. Will get a valid filtered result next time (if two scq pass then)
			s_filteredRPM[channel] = 0.0f;	//reset `s_filteredRPM` (invalidate it)
			return;
		}// here we had a valid reference edge before. now up to computation

		//core calc
		currRPM = (speedSign[channel] ? -1.0f : 1.0f) *
				  ((60.0f * 1000000.0f) / 
				   ((float)ENC_PULSES_PER_REV * (float)deltaTimeUs));

		//lag filter
		if(s_filteredRPM[channel] == 0.0f)
		{// re-seeds after a standstill where we reset the filter (see `deltaTimeUs > ENC_IC_EDGE_TIMEOUT_US` branch above)
			s_filteredRPM[channel] = currRPM;
		}
		else
		{// apply lag filter
			s_filteredRPM[channel] += ENC_IC_LAG_FILTER_COEF * (currRPM - s_filteredRPM[channel]);
		}
	}
	else if(wen == 1U) // just output the filtered RPM here
	{
		uint8_t i;
		for(i = 0; i < 2; i++)
		{
			//scq
			if(s_filteredRPM[i] == 0.0f)
			{// can't see a valid result yet
				// do not update the output and check the next one
				continue;
			}

			// safe to update with a valid filtered value
			speedRPM[i] = s_filteredRPM[i];
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
		float    encRPM[2] = {0.0f, 0.0f};
		float	 icRPM[2]  = {0.0f, 0.0f};
		uint16_t encCPS[2] = {0U, 0U};

		Enc_SampleSpeedByEnc(encRPM, encCPS, nowMs, 1U);
		Enc_SampleSpeedByIC(icRPM, (void *)0, 1U);

		tg_encRPM = encRPM[0];
		tg_icRPM  = icRPM[0];

		/* Routing method */
		if(1)
		{
			speedRPM[0] = icRPM[0];
			speedRPM[1] = icRPM[1];
		}

		s_nextSampMs = nowMs + ENC_SPEED_SAMP_INTVL_MS;
		return 1;
	}
	else
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
		Enc_SampleSpeedByIC((void *)0, 0U, 0U);
	}

	if(TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
	{// R side sent the pulse
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		Enc_SampleSpeedByIC((void *)0, 1U, 0U);
	}
}

