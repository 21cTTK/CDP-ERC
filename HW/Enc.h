#ifndef __ENC_H
#define __ENC_H

#include <stdint.h>

/* TIM3 IC Timer Macro */
//tunables
//! param @ref PWM-Motor.c should use the same set of param to make PWM work
#define ENC_IC_PSC_DIV	72U		
#define ENC_IC_PWM_ARR	1000U
#define ENC_IC_TICK_US	1U		
//derivative constants
#define ENC_IC_WRAP_US	(ENC_IC_PWM_ARR * ENC_IC_TICK_US)	//!< us per CNT wrap (65536 us)

void Enc_Init(void);
uint8_t Enc_SampleSpeed(float *speed, uint32_t nowMs);
void Enc_ISR(void);

#endif//__ENC_H
