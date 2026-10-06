#ifndef __PID_H
#define __PID_H

#include <stdint.h>

void PID_Init(void);
float* PID_Update(float* actualRPM, float* targetRPM);
void PID_SetGains(float Kp, float Ki);

#endif//__PID_H
