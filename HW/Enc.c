#include "stm32f10x.h"
#include "Enc.h"

#define ENC_SPEED_SAMP_INTVL_MS  	50
#define ENC_CNT_ZERO_BASE   32768 // 1<<15
#define ENC_LINES 			11
#define ENC_MULT			4	  // depends on the config of TIM Encoder Interface; typically `TIM_EncoderMode_TI12` ==> `4`
#define ENC_GEAR_RATIO  	9.6

/* Speed sampling method routing; see @ref `Enc_SampleSpeed()` */
#define ENC_SAMPLE_ROUTING_THRESHOLD  300.0f	// [RPM] at/above this the count-difference method is authoritative
#define ENC_SAMPLE_ROUTING_HYST       30.0f		// [RPM] hysteresis band around `ENC_SAMPLE_ROUTING_THRESHOLD`
#define ENC_SAMPLE_SWITCH_MAX_JUMP    150.0f	// [RPM] largest step accepted on the 1st sample after a handoff

/* TIM3 input capture(pulse width) parameters; the TIM3 time base itself belongs to
   PWM-Motor.c: PSC = 72-1, ARR = 1000-1  ==>  1 us per tick, `CNT` wraps every 1000 ticks */
#define ENC_IC_TICK_US          1U								// [us] duration of one TIM3 tick
#define ENC_IC_CNT_MOD          1000U							// TIM3 `CNT` modulus (ARR+1); `CNT` wraps at 1000
#define ENC_IC_PULSES_PER_REV   (ENC_LINES * ENC_GEAR_RATIO)	// signal periods per output-shaft rev, ONE channel
#define ENC_IC_MIN_HALF_TICKS   40U								// [ticks] shorter edge spacing is noise, not an edge

volatile uint32_t g_CNT;

/// @brief One TIM3 input-capture channel's pulse-width measurement state.
/// @note - written by @ref `Enc_ISR()`, read by @ref `Enc_SampleSpeedByIC()`
typedef struct
{
	volatile uint32_t lastExt;		///< wrap-extended timestamp of the last edge [ticks]
	volatile uint32_t halfTicks;	///< last edge-to-edge interval (half a signal period) [ticks]
	volatile uint32_t periodTicks;	///< last complete signal period (half + half) [ticks]
	volatile uint32_t seq;			///< completed-period counter; a change means "fresh data"
	volatile uint8_t  haveFirst;	///< `1` once a first edge has been timestamped
	volatile uint8_t  haveHalf;		///< `1` once a first half period is latched
	volatile uint8_t  falling;		///< armed capture edge of this channel; `1` = falling
} Enc_ICChan;

/// TIM3 input capture channels: `[0]` = CH1(L source), `[1]` = CH2(R source)
static Enc_ICChan s_icChan[2];
/// TIM3 `CNT` wrap count (1 ms per wrap) since @ref `Enc_Init()`; the IC time base's high word
static volatile uint32_t s_icWrapCnt;
/// `s_icChan[n].seq` as consumed by the previous @ref `Enc_SampleSpeed()` sample
static uint32_t s_icSeqDone[2];

/* Helper Functions Section */

/// @brief `fabsf()` by hand: keeps <math.h> out of the sampling path
/// @param val value to take the magnitude of
/// @return - `|val|`
static float Enc_Abs(float val)
{
	return (val < 0.0f) ? -val : val;
}

/// @brief Read in curr CNT and reset it to `ENC_CNT_ZERO_BASE`.
/// @note - real CNT ticks are calculated based on delta with `ENC_CNT_ZERO_BASE` to prevent ovf/udf
/// @note - used by both @ref `Enc_SampleSpeedByCnt` and @ref `Enc_SampleSpeedByIC`
/// @param cnt value storage array
static void Enc_GetCNT(uint16_t *cnt)
{
	cnt[0] = TIM_GetCounter(TIM1);
	cnt[1] = TIM_GetCounter(TIM2);
	TIM_SetCounter(TIM1, ENC_CNT_ZERO_BASE);
	TIM_SetCounter(TIM2, ENC_CNT_ZERO_BASE);
}

/// @brief Arm the opposite capture edge of a TIM3 channel, so HIGH and LOW times are both measured.
/// @param ch channel index; `0` = TIM3_CH1, `1` = TIM3_CH2
/// @note - the SPL has no `TIM_ICxPolarityConfig()`; capture and output polarity share the
///         `CCER` CCxP bit, so the OC setter (which flips exactly that bit) is used
static void Enc_ICArmOppositeEdge(uint8_t ch)
{
	Enc_ICChan *chan = &s_icChan[ch];

	chan->falling ^= 1U;
	if(ch == 0U)
		TIM_OC1PolarityConfig(TIM3, chan->falling ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising);
	else
		TIM_OC2PolarityConfig(TIM3, chan->falling ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising);
}

/// @brief Fold one latched TIM3 capture into the pulse-width measurement of a channel.
/// @param ch channel index; `0` = TIM3_CH1, `1` = TIM3_CH2
/// @param cap `CCRx` as latched by the edge (the `CNT` value at the edge)
/// @param cntNow `CNT` snapshot taken after that `CCRx` was read, see @ref `Enc_ISR()`
static void Enc_ICLatch(uint8_t ch, uint16_t cap, uint16_t cntNow)
{
	Enc_ICChan *chan = &s_icChan[ch];
	uint32_t ext = (s_icWrapCnt * ENC_IC_CNT_MOD) + cap;
	uint32_t halfTicks;

	/* `CNT` wrapped between the edge and this ISR run while `s_icWrapCnt` already counts
	 * that wrap: the edge belongs to the wrap period before it, so the stamp steps one
	 * wrap back. This is why the wrap is accounted instead of resetting `CNT`: `CNT` is
	 * shared with the CH3/CH4 motor PWM and must run free. */
	if(cap > cntNow)
		ext -= ENC_IC_CNT_MOD;

	if(chan->haveFirst == 0U)
	{// 1st edge can only establish the time base; no interval is known yet
		chan->haveFirst = 1U;
		chan->lastExt = ext;
	}
	else
	{
		halfTicks = ext - chan->lastExt;
		chan->lastExt = ext;

		if(halfTicks < ENC_IC_MIN_HALF_TICKS)
		{// too short to be a real edge: re-base and drop the period being formed
			chan->haveHalf = 0U;
		}
		else if(chan->haveHalf == 0U)
		{// 1st half of a signal period: HIGH or LOW time
			chan->halfTicks = halfTicks;
			chan->haveHalf = 1U;
		}
		else
		{// 2nd half: HIGH + LOW = one full signal period, independent of the duty cycle
			chan->periodTicks = chan->halfTicks + halfTicks;
			chan->halfTicks = halfTicks;
			chan->seq++;
		}
	}

	Enc_ICArmOppositeEdge(ch);
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

	//TIM IC(TIM3); SMCR keeps its reset value (SMS = 0, internal clock) and `CNT` is never
	//reset, so the timer runs free -- both the pulse width time base below and the CH3/CH4
	//PWM of PWM-Motor.c depend on that. The trigger select is inert while SMS = 0; enabling
	//slave mode would zero CNT on edges and destroy the time base.
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Disable);

	//TIM Encoder Interface
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	//Pulse width measurement state + IRQ(TIM3): CHx captures and the CNT wrap(update)
	s_icWrapCnt = 0U;
	for(int ch = 0; ch < 2; ch++)
	{
		s_icChan[ch].lastExt = 0U;
		s_icChan[ch].halfTicks = 0U;
		s_icChan[ch].periodTicks = 0U;
		s_icChan[ch].seq = 0U;
		s_icChan[ch].haveFirst = 0U;
		s_icChan[ch].haveHalf = 0U;
		s_icChan[ch].falling = 0U;
		s_icSeqDone[ch] = 0U;
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2);	// e.g. the UG request of TIM_TimeBaseInit above
	TIM_ITConfig(TIM3, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

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

/* Sampling Methods Section */

/// @brief Count-difference method: a rough RPM from the encoder counts seen by TIM1/TIM2.
/// @param speedRPM float array to store the RPM values
/// @param elapsedMs time since the previous sample, i.e. since `Enc_GetCNT()` zeroed the counters
/// @note - Assumption: CNT do not ovf or udf (ARR/2@32768); mind your `ENC_SPEED_SAMP_INTVL_MS`
/// @return - `1` if speed was updated
/// @return - `0` otherwise
static uint8_t Enc_SampleSpeedByCnt(float *speedRPM, uint32_t elapsedMs)
{
	uint16_t nowCNT[2]    = {0, 0};
	uint16_t deltaCNT[2]  = {0, 0};
	uint8_t  speedSign[2] = {0, 0};

	Enc_GetCNT(nowCNT);	// also reset CNT
	g_CNT = nowCNT[0];

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
				   ((ENC_LINES * ENC_GEAR_RATIO * ENC_MULT) *elapsedMs)
				  ) * 1000 * 60; // L speed

	speedRPM[1] = (speedSign[1] ? -1 : 1) *
				  (deltaCNT[1] /
				   ((ENC_LINES * ENC_GEAR_RATIO * ENC_MULT) * elapsedMs)
				  ) * 1000 * 60; // R speed

	return 1;
}

/// @brief TIM3 input-capture method: RPM from the measured period of the encoder pulse train.
/// @param speedRPM float array to store the RPM values
/// @param fresh per-side flags: `1` = a complete pulse period arrived since the previous sample
/// @note - serviced by @ref `Enc_ISR()`; the period used is the sum of two consecutive
///         edge-to-edge intervals (HIGH + LOW time), so a drifting duty cycle cannot bias it
/// @note - RPM = 60e6 [us/min] / (period [us] * `ENC_IC_PULSES_PER_REV`), i.e. the very same
///         quantity the count method divides by `ENC_LINES * ENC_MULT * ENC_GEAR_RATIO`
/// @note - the IC path measures the period only, so the rotation *sign* is taken from the
///         TIM1/TIM2 encoder ticks (the same encoder phases); they are consumed here as well
///         so that the counters can never run away while the IC method is in charge
/// @note - a side without a fresh period is turning slower than the sample interval can
///         resolve (or stands still) and reads 0 RPM, so no stale value is ever handed over
/// @return - `1` if speed was updated
/// @return - `0` otherwise
static uint8_t Enc_SampleSpeedByIC(float *speedRPM, const uint8_t *fresh)
{
	static uint8_t s_speedSign[2] = {0, 0};	// last known direction; held while a side is too slow to tell
	uint16_t nowCNT[2] = {0, 0};
	uint8_t  updated = 0U;
	uint8_t  ch;

	Enc_GetCNT(nowCNT);	// also reset CNT
	g_CNT = nowCNT[0];

	for(ch = 0U; ch < 2U; ch++)
	{
		float periodTicks;

		if(nowCNT[ch] != ENC_CNT_ZERO_BASE)
			s_speedSign[ch] = (nowCNT[ch] < ENC_CNT_ZERO_BASE) ? 1U : 0U;

		if(fresh[ch] == 0U)
		{// no pulse period closed since the previous sample
			speedRPM[ch] = 0.0f;
			continue;
		}

		updated = 1U;
		periodTicks = (float)s_icChan[ch].periodTicks;

		speedRPM[ch] = (s_speedSign[ch] ? -1.0f : 1.0f) *
					   (60.0f * 1000000.0f) /
					   (periodTicks * (float)ENC_IC_TICK_US * (float)ENC_IC_PULSES_PER_REV);
	}

	return updated;
}

/// @brief Sample both wheel speeds, routed to one of the two measurement methods.
/// @param speedRPM float array to store the RPM values
/// @param nowMs current system time in Ms
/// @note - `ENC_SAMPLE_ROUTING_THRESHOLD` splits the two methods: the count-difference method
///         is authoritative at/above it (its fixed ±1 count quantum is small next to the
///         result), while below `ENC_SAMPLE_ROUTING_THRESHOLD - ENC_SAMPLE_ROUTING_HYST`
///         the TIM3 pulse-width method takes over (it resolves slow rotation that the 50 ms
///         count window cannot). The hysteresis band in between keeps whichever method is
///         active, so a speed resting on the threshold cannot make the routing chatter.
/// @note - the IC method is only taken while TIM3 actually delivers pulse periods: an idle
///         capture input (wheel slower than the sample interval, or input not wired) hands
///         the sample back to the count method, which always has an answer
/// @note - both methods consume the TIM1/TIM2 counters once per sample, so the handoff never
///         uses a stale interval, and the 1st sample of a freshly routed method is dropped
///         if it jumps by more than `ENC_SAMPLE_SWITCH_MAX_JUMP` [RPM]: the consumer then
///         keeps its previous value instead of being handed a handoff artifact
/// @return - `1` if speed was updated
/// @return - `0` otherwise
uint8_t Enc_SampleSpeed(float *speedRPM, uint32_t nowMs)
{
	/* `0` = count-difference (TIM1/TIM2 encoder ticks), `1` = pulse width (TIM3 input capture) */
	static uint8_t  s_method      = 0U;
	static uint8_t  s_firstSample = 1U;
	static uint8_t  s_switchGuard = 0U;		// check the handoff jump guard on the next produced sample
	static uint32_t s_nextSampMs  = 0U;
	static float    s_lastRPM[2]  = {0.0f, 0.0f};

	uint32_t elapsedMs = nowMs - s_nextSampMs + ENC_SPEED_SAMP_INTVL_MS;
	uint32_t icSeq[2];
	uint8_t  icFresh[2];
	uint8_t  icAlive;
	uint8_t  updated;
	float    sample[2];
	float    fastest;
	uint16_t nowCNT[2] = {0, 0};

	if(s_firstSample)
	{// discard first sample (but still do the routine)
		Enc_GetCNT(nowCNT);	// also reset CNT
		icSeq[0] = s_icChan[0].seq;	// the IC freshness reference starts counting from here
		icSeq[1] = s_icChan[1].seq;
		s_icSeqDone[0] = icSeq[0];
		s_icSeqDone[1] = icSeq[1];
		s_nextSampMs = nowMs + ENC_SPEED_SAMP_INTVL_MS;
		s_firstSample = 0U;
		return 0;
	}

	if(nowMs < s_nextSampMs)
		return 0;

	s_nextSampMs = nowMs + ENC_SPEED_SAMP_INTVL_MS;

	/* Pulse periods that closed since the previous sample. The reference is consumed on
	 * every sample, no matter which method is in charge, so `icFresh[]` always means
	 * "arrived during the last `ENC_SPEED_SAMP_INTVL_MS`". The counters are snapshotted
	 * once and that same snapshot is consumed: reading them twice would let the ISR close
	 * a period between "is it fresh?" and "remember it as consumed", silently dropping it
	 * (and reporting that side as 0 RPM). */
	icSeq[0] = s_icChan[0].seq;
	icSeq[1] = s_icChan[1].seq;
	icFresh[0] = (uint8_t)(icSeq[0] != s_icSeqDone[0]);
	icFresh[1] = (uint8_t)(icSeq[1] != s_icSeqDone[1]);
	s_icSeqDone[0] = icSeq[0];
	s_icSeqDone[1] = icSeq[1];
	icAlive = (uint8_t)(icFresh[0] | icFresh[1]);

	/* Method routing, judged on the last reported speed so that the decision does not
	 * depend on which method produced it */
	fastest = Enc_Abs(s_lastRPM[0]);
	if(Enc_Abs(s_lastRPM[1]) > fastest)
		fastest = Enc_Abs(s_lastRPM[1]);

	if(s_method == 0U)
	{
		if((fastest <= (ENC_SAMPLE_ROUTING_THRESHOLD - ENC_SAMPLE_ROUTING_HYST)) && (icAlive != 0U))
		{
			s_method = 1U;
			s_switchGuard = 1U;
		}
	}
	else if((fastest >= (ENC_SAMPLE_ROUTING_THRESHOLD + ENC_SAMPLE_ROUTING_HYST)) || (icAlive == 0U))
	{
		s_method = 0U;
		s_switchGuard = 1U;
	}

	if(s_method == 1U)
		updated = Enc_SampleSpeedByIC(sample, icFresh);
	else
		updated = Enc_SampleSpeedByCnt(sample, elapsedMs);

	if(updated == 0U)
		return 0;	// nothing to hand over; the consumer keeps its previous values

	if(s_switchGuard != 0U)
	{// first sample of a freshly routed method: refuse an out-of-family handoff
		s_switchGuard = 0U;
		if((Enc_Abs(sample[0] - s_lastRPM[0]) > ENC_SAMPLE_SWITCH_MAX_JUMP) ||
		   (Enc_Abs(sample[1] - s_lastRPM[1]) > ENC_SAMPLE_SWITCH_MAX_JUMP))
			return 0;
	}

	speedRPM[0]  = sample[0];
	speedRPM[1]  = sample[1];
	s_lastRPM[0] = sample[0];
	s_lastRPM[1] = sample[1];
	return 1;
}

/* ISR Section */

/// @brief TIM3 ISR body: CH1/CH2 pulse-width captures and `CNT` wrap accounting.
/// @note - called from @ref `TIM3_IRQHandler()` in `stm32f10x_it.c`
void Enc_ISR(void)
{
	uint16_t cap[2] = {0, 0};
	uint16_t cntNow;
	uint16_t cntCheck;
	uint8_t  uifPending;
	uint8_t  captured = 0U;

	/* Latch the pending captures first: `CCRx` is read (and only then cleared) while its
	 * flag is set, and before the `CNT` snapshot below -- `Enc_ICLatch()` decides "edge
	 * before the latest wrap" or "edge after it" from `cap > cntNow`, which is only
	 * meaningful if the capture was latched before `cntNow` was read.
	 * `TIM_ClearITPendingBit()` writes 1s into every SR flag bit it is not asked to clear,
	 * so servicing CH1 leaves CH2 (and the update flag) pending: simultaneous edges on
	 * both channels are kept. */
	if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{
		cap[0] = TIM_GetCapture1(TIM3);
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		captured |= 1U;
	}
	if(TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
	{
		cap[1] = TIM_GetCapture2(TIM3);
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		captured |= 2U;
	}

	/* `CNT` and the wrap flag it is judged against must come from ONE consistent sample:
	 * this retry only exits once `CNT` did not tick across the sequence, which proves that
	 * no wrap can have slipped in between the two reads and invalidated the `cap > cntNow`
	 * test of `Enc_ICLatch()`. (`CNT` ticks once per us, so it settles in an iteration or
	 * two.) */
	do
	{
		cntNow     = TIM_GetCounter(TIM3);
		uifPending = (uint8_t)(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET);
		cntCheck   = TIM_GetCounter(TIM3);
	} while(cntCheck != cntNow);

	/* A `CNT` wrap (`ENC_IC_CNT_MOD` ticks = 1 ms @ 1 us/tick) is the high word of the IC
	 * time base: it MUST be counted before the captures are stamped below. */
	if(uifPending != 0U)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		s_icWrapCnt++;
	}

	if((captured & 1U) != 0U)
		Enc_ICLatch(0U, cap[0], cntNow);
	if((captured & 2U) != 0U)
		Enc_ICLatch(1U, cap[1], cntNow);
}
