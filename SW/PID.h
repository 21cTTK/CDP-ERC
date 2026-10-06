#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/// @brief number of stages (staircase sub-steps) a new target is split into
#define PID_RAMP_DEFAULT_STAGES     10U

void PID_Init(void);
float* PID_Update(float* actualRPM, float* targetRPM);
void PID_SetGains(float Kp, float Ki);
void PID_SetRamp(uint32_t dtMs, uint16_t nStages);

#endif//__PID_H
