#ifndef __LEDS_H
#define __LEDS_H

#include "stm32f10x.h"
#include <stdint.h>

typedef enum __LEDS_DEF_ENUM
{
	LED_MAIN = 0, 	// Used for: Loop Indicator, Init Indicator
	LED_MSG_RX,    	// Used for: RC module msg received indicator
	LED_CMD_EXEC,  	// Used for: RC module cmd executed indicator
	LED_I2C_ERROR,  // Used for: I2C error indicator
	LED_COUNT		// should match the number of used LEDs
} LEDslot;

void LED_Init(void);
void LED_TurnON(LEDslot slot);
void LED_TurnOFF(LEDslot slot);
void LED_CheckFlicker_SysTick(uint32_t nowMs);
void LED_SetFlicker(uint32_t nowMs, LEDslot slot, uint8_t times, uint16_t intervalMs);
void LED_StopFlicker(LEDslot slot);
uint8_t LED_IsON(LEDslot slot);
uint8_t LED_IsFlickering(LEDslot slot);

#endif//__LEDS_H
