#ifndef __LEDS_H
#define __LEDS_H

#include "stm32f10x.h"
#include <stdint.h>

typedef enum __LEDS_DEF_ENUM
{
	LED_MAIN = 0, // Used for: Loop Indicator, Init Indicator
	LED_MSG,    // Used for: RC module msg received indicator
	LED_COUNT
} LEDslot;

typedef struct __LEDS_DEF_STRUCT
{
	GPIO_TypeDef *port;
	uint16_t 	  pin;
} LEDdef;

static const LEDdef g_keysDef[LED_COUNT] =
{
	[LED_MAIN] = {GPIOC, GPIO_Pin_13},
	[LED_MSG]  = {GPIOA, GPIO_Pin_6}
};

void LED_Init(void);
void LED_TurnON(LEDslot slot);
void LED_TurnOFF(LEDslot slot);
void LED_CheckFlicker(uint32_t nowMs);
void LED_SetFlicker(uint32_t nowMs, LEDslot slot, uint8_t times, uint16_t intervalMs);
void LED_StopFlicker(LEDslot slot);
uint8_t LED_IsON(LEDslot slot);
uint8_t LED_IsFlickering(LEDslot slot);

#endif//__LEDS_H
