#include "LEDs.h"
typedef struct __LEDS_DEF_STRUCT
{
	GPIO_TypeDef *port;
	uint16_t 	  pin;
} LEDdef;

static const LEDdef g_LEDsDef[LED_COUNT] =
{
	[LED_MAIN]     = {GPIOC, GPIO_Pin_13},
	[LED_MSG_RX]   = {GPIOA, GPIO_Pin_6},
	[LED_CMD_EXEC] = {GPIOA, GPIO_Pin_7}
};

static uint8_t  g_toggleTimes[LED_COUNT]      = {0U};	//tracking remaining toggle times for each LED
static uint32_t g_nextToggleMs[LED_COUNT]     = {0U};	//tracking next toggle time for each LED
static uint32_t g_toggleIntervalMs[LED_COUNT] = {0U};	//record set interval for each LED
static uint8_t  g_flickering[LED_COUNT]       = {0U};	//tracking if an LED is flickering

/// @brief Check if the current time has reached the specified deadline.
/// @param now The current time.
/// @param ddl The time to check against.
/// @retval `1` for `now` >= `ddl`; `0` for `now` < `ddl`.
static uint8_t LED_FlickerTimeReached(uint32_t now, uint32_t ddl)
{
	if(!ddl)
		return 0U;

	int32_t diff = now - ddl;
	return (uint8_t)(diff >= 0);
}

void LED_Init(void)
{
	uint8_t i;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
						   RCC_APB2Periph_GPIOB |
						   RCC_APB2Periph_GPIOC , ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	for(i = 0U; i < LED_COUNT; i++)
	{
		GPIO_InitStructure.GPIO_Pin = g_LEDsDef[i].pin;
		GPIO_Init(g_LEDsDef[i].port, &GPIO_InitStructure);

		//Init OFF
		GPIO_SetBits(g_LEDsDef[i].port, g_LEDsDef[i].pin);
	}
}

void LED_TurnON(LEDslot slot)
{
	if (slot < LED_COUNT)
		GPIO_ResetBits(g_LEDsDef[slot].port, g_LEDsDef[slot].pin);
}

void LED_TurnOFF(LEDslot slot)
{
	if (slot < LED_COUNT)
		GPIO_SetBits(g_LEDsDef[slot].port, g_LEDsDef[slot].pin);
}

/// @param nowMs: Initial timestamp for the flicker; starts flicker after `intervalMs` of this
/// @param times: The number of flickers (ON/OFF cycles) to perform.
/// @param intervalMs: The interval between toggles in milliseconds (ON state and OFF state BOTH at `intervalMs`)
/// @note the end state is the same as its initial state; set ON/OFF @ref (`void LED_TurnON(LEDslot slot)`)
/// 	  before calling this function to guarantee a desired end state.
void LED_SetFlicker(uint32_t nowMs, LEDslot slot, uint8_t times, uint16_t intervalMs)
{
	if ((slot < LED_COUNT) && (times > 0U) && (intervalMs > 0U))
	{
		g_toggleTimes[slot] = times * 2U;			// Each ON/OFF counts as one toggle
		g_nextToggleMs[slot] = nowMs + intervalMs;
		g_toggleIntervalMs[slot] = intervalMs;
		g_flickering[slot] = 1U;
	}
}

/// @note End state not guaranteed; set ON/OFF @ref (`void LED_TurnON(LEDslot slot)`)
/// 	  after calling this function to achieve the desired end state.
void LED_StopFlicker(LEDslot slot)
{
	if (slot < LED_COUNT)
	{
		g_toggleTimes[slot] = 0U;
		g_flickering[slot] = 0U;
	}
}

/// @brief `Update()` fn of LEDs module; Check and update the flickering state of each LED.
/// @param nowMs The current time in milliseconds.
void LED_CheckFlicker(uint32_t nowMs)
{
	uint8_t i;

	for(i = 0U; i < LED_COUNT; i++)
	{
		if(g_toggleTimes[i] && g_flickering[i])
		{
			if(LED_FlickerTimeReached(nowMs, g_nextToggleMs[i]))
			{
				if(LED_IsON((LEDslot)i))
					LED_TurnOFF((LEDslot)i);
				else
					LED_TurnON((LEDslot)i);

				g_toggleTimes[i]--;
				g_nextToggleMs[i] = nowMs + g_toggleIntervalMs[i];
			}
			//else;
		}
		//else;
	}
}

uint8_t LED_IsON(LEDslot slot)
{
	if (slot < LED_COUNT)
		return (uint8_t)(
			GPIO_ReadInputDataBit(g_LEDsDef[slot].port, g_LEDsDef[slot].pin)
			== Bit_RESET);

	return 0U;
}

uint8_t LED_IsFlickering(LEDslot slot)
{
	if (slot < LED_COUNT)
		return g_flickering[slot];

	return 0U;
}
