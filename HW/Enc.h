#ifndef __ENC_H
#define __ENC_H

#include <stdint.h>

void Enc_Init(void);
uint8_t Enc_SampleSpeed(float *speed, uint32_t nowMs);

#endif//__ENC_H
