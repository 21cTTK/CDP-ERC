#ifndef __PID_H
#define __PID_H

#include <stdint.h>

void PID_Init(void);
float* PID_Ramp(void);
float* PID_Update(float* actualRPM, float* targetRPM, uint8_t shouldStop);
void PID_SetGains(float Kp, float Ki);
void PID_GetActualRPM(float* actualRPM);
void PID_SetTargetRPM(float* targetRPM);

#endif//__PID_H
