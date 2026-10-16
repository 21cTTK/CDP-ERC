#ifndef __KEYS_H
#define __KEYS_H

#include "stm32f10x.h"
#include <stdint.h>

typedef enum __KEY_EV_ENUM
{
	KEY_EV_LED_ON = 0,
    KEY_EV_COUNT                // should match the number of used Keys
} KeyEvents;

void Key_Init(void);
void Key_UniISR(uint32_t line);
void Key_CheckState_SysTick(void);
// void Key_GetEvents(uint8_t **events);
void Key_GetEvents(uint8_t *events);

#endif//__KEYS_H
