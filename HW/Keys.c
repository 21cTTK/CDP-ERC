#include "Keys.h"

#define KEY_DEBOUNCE_MS     30U
#define KEY_RLS_STABLE_MS   10U

static uint8_t g_CurrKeyEV[KEY_EV_COUNT];

typedef enum __KEY_STATE_ENUM
{
	KEY_STATE_IDLE = 0,
	KEY_STATE_DEBOUNCE,
	KEY_STATE_WAIT_RLS
} KeyState;

typedef struct __KEYS_DEF_STRUCT
{
	GPIO_TypeDef *port;
	uint32_t line;				//EXTI line; also used for GPIO pin
	volatile KeyState state;
	volatile uint8_t counter;
} KeysDef;

/// @brief keys pinout and separate states and count-downs
static KeysDef g_KeysDef[KEY_EV_COUNT] =
{
	[KEY_EV_LED_ON] = {GPIOA, EXTI_Line4, KEY_STATE_IDLE, 0U}
};

void Key_Init(void)
{
	GPIO_InitTypeDef gpio;
	EXTI_InitTypeDef exti;
	NVIC_InitTypeDef nvic;
	uint8_t i;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
						   RCC_APB2Periph_GPIOB	|
						   RCC_APB2Periph_GPIOC |
						   RCC_APB2Periph_AFIO, ENABLE);

	// GPIO
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	for (i = 0U; i < KEY_EV_COUNT; i++)
	{
		gpio.GPIO_Pin = g_KeysDef[i].line;
		GPIO_Init(g_KeysDef[i].port, &gpio);
	}

	//EXTI
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Falling;
	exti.EXTI_LineCmd = ENABLE;
	for (i = 0U; i < KEY_EV_COUNT; i++)
	{
		GPIO_EXTILineConfig(g_KeysDef[i].port, g_KeysDef[i].line);
		exti.EXTI_Line |= g_KeysDef[i].line;
	}

	EXTI_Init(&exti);

	//NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	nvic.NVIC_IRQChannelPreemptionPriority = 1U;
	nvic.NVIC_IRQChannelSubPriority = 0U;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	for(i = 0U; i < KEY_EV_COUNT; i++)
	{
		if (g_KeysDef[i].line == EXTI_Line0)
			nvic.NVIC_IRQChannel = EXTI0_IRQn;
		else if (g_KeysDef[i].line == EXTI_Line1)
			nvic.NVIC_IRQChannel = EXTI1_IRQn;
		else if (g_KeysDef[i].line == EXTI_Line2)
			nvic.NVIC_IRQChannel = EXTI2_IRQn;
		else if (g_KeysDef[i].line == EXTI_Line3)
			nvic.NVIC_IRQChannel = EXTI3_IRQn;
		else if (g_KeysDef[i].line == EXTI_Line4)
			nvic.NVIC_IRQChannel = EXTI4_IRQn;
		else if (g_KeysDef[i].line >= EXTI_Line5 && g_KeysDef[i].line <= EXTI_Line9)
			nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
		else
			continue;

		NVIC_Init(&nvic);
	}

	//Init Cleanup
	for (i = 0U; i < KEY_EV_COUNT; i++)
	{
		g_CurrKeyEV[i] = 0U;	// No EV active
		g_KeysDef[i].state = KEY_STATE_IDLE;
		g_KeysDef[i].counter = 0U;
		EXTI_ClearITPendingBit(g_KeysDef[i].line);
	}
}

void Key_UniISR(uint32_t line)
{
	uint8_t i;

	// Find the key that matches the line
	// set to Debounce state, and disable the line until debounce is complete.
	for (i = 0U; i < KEY_EV_COUNT; i++)
	{
		if (g_KeysDef[i].line == line)
		{
			/// This EXTI IT line is Disabled to ensure safe debounce; will be EN in @ref Key_SysTickCheck(void)
			EXTI->IMR &= ~line;
			g_KeysDef[i].counter = KEY_DEBOUNCE_MS;
			g_KeysDef[i].state = KEY_STATE_DEBOUNCE;
			return;
		}
	}
}

void Key_CheckState_SysTick(void)
{
	uint8_t i;

	for (i = 0U; i < KEY_EV_COUNT; i++)
	{
		switch (g_KeysDef[i].state)
		{
		case KEY_STATE_DEBOUNCE:
			// Wait `KEY_DEBOUNCE_MS` and re-check the key state
			// EXTI IT is disabled; Enable it when key is detected as rls after `KEY_DEBOUNCE_MS`
			// (means not taken as a valid press; ready to be pressed again)
			if(g_KeysDef[i].counter != 0U)
			{
				g_KeysDef[i].counter--;
			}
			else if(g_KeysDef[i].counter == 0U)
			{
				if(GPIO_ReadInputDataBit(g_KeysDef[i].port, g_KeysDef[i].line) == Bit_RESET)
				{
					g_CurrKeyEV[i] = 1U;	// confirmed press; set up Key EV
					g_KeysDef[i].state = KEY_STATE_WAIT_RLS;
				}
				else
				{
					EXTI->IMR |= g_KeysDef[i].line;	// Enable this EXTI IT line
					g_KeysDef[i].state = KEY_STATE_IDLE;
				}
			}
			
			break;
		case KEY_STATE_WAIT_RLS:
			// Ensure `KEY_RLS_STABLE_MS` time long of actual rls (stable rls)
			// EXTI IT is disabled; Enable it when a stable released is detected (ready to be pressed again)
			if(GPIO_ReadInputDataBit(GPIOA, g_KeysDef[i].line) == Bit_SET)
			{// possible rls
				if (g_KeysDef[i].counter < KEY_RLS_STABLE_MS)
				{
					g_KeysDef[i].counter++;
				}
				else if (g_KeysDef[i].counter >= KEY_RLS_STABLE_MS)
				{
					g_KeysDef[i].counter = 0U;
					g_KeysDef[i].state = KEY_STATE_IDLE;
					EXTI->IMR |= g_KeysDef[i].line;	// Enable this EXTI IT line
				}
			}
			else
			{// still pressed
				g_KeysDef[i].counter = 0U;
			}

			break;
		case KEY_STATE_IDLE:
			break;
		default:
			break;
		}
	}
}

/// @brief `Update()` fn of Keys module; Check whether there's a event or not.
/// @param events Pointer to the array to store the key events
void Key_GetEvents(uint8_t *events)
{
	uint8_t i;
	for(i = 0; i < KEY_EV_COUNT; i++)
	{
		events[i] = g_CurrKeyEV[i];
	}

	//!G: assuming no persistent EV (all existing EV is one-off as to the loop)
	for(i = 0U; i < KEY_EV_COUNT; i++)
	{
		g_CurrKeyEV[i] = 0U;	// Clear the EV after reading
	}
}
