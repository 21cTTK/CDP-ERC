#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct __PID_PARAMS_STRUCT {
    float actual;
    float setPoint;
    float errorInt;
    float output;
    float rampPoint;
    /* Macros */
    float Kp;
    float Ki;
    float rampStep;
    float iTermClamp;
    float pTermClamp;
} PidParams_t;

void PID_Ramp(PidParams_t* params);
void PID_Update(PidParams_t* params, uint8_t shouldStop);
void PID_WriteGains(PidParams_t* paramsDst, float Kp, float Ki);
void PID_WriteConfig(PidParams_t* paramsDst, float rampStep, float iTermClamp, float pTermClamp);
void PID_WriteParams(PidParams_t* paramsDst, const PidParams_t* paramsSrc);
void PID_CopyParams(PidParams_t* paramsDst, const PidParams_t* paramsSrc);
void PID_ClearParams(PidParams_t* params);

#endif//__PID_H
