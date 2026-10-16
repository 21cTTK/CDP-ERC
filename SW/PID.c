#include "stm32f10x.h"                  // Device header
#include "Motion.h"
#include "PWM-Motor.h"
#include "PID.h"
#include <math.h>

#define PID_PTERM_CLAMP     MOTOR_PWM_DUTY_MAX
#define PID_ITEMR_CLAMP     110.0f
#define PID_RAMP_STEP_SIZE  64.0f

void PID_Ramp(PidParams_t* params)
{
    if(params->rampPoint < params->setPoint)
    {//forward ramping
        params->rampPoint += PID_RAMP_STEP_SIZE;
        if(params->rampPoint > params->setPoint)
            params->rampPoint = params->setPoint;
    }
    else if(params->rampPoint > params->setPoint)
    {//backward ramping
        params->rampPoint -= PID_RAMP_STEP_SIZE;
        if(params->rampPoint < params->setPoint)
            params->rampPoint = params->setPoint;
    }
}

void PID_Update(PidParams_t* params, uint8_t shouldStop)
{
    //if isStop is true, reset the integrated error
    if (shouldStop == 1) {
        PID_ClearParams(params);
        return;
    }

    //compute curr error
    float currError = params->rampPoint - params->actual;

    //integrated error
    params->errorInt += currError;

    //clamp integrated error
    float intClamp = PID_ITEMR_CLAMP / params->Ki;
    if(fabsf(params->errorInt) > intClamp)
    {
        params->errorInt = intClamp * (params->errorInt > 0 ? 1 : -1);
    }

    params->output = params->Kp * currError +
                     params->Ki * params->errorInt;

    //output clamp control output to [-100, 100]
    if (fabsf(params->output) > PID_PTERM_CLAMP)
    {
        params->output = PID_PTERM_CLAMP * (params->output > 0 ? 1 : -1);
    }
}

void PID_WriteGains(PidParams_t* paramsDst, float Kp, float Ki)
{
    if(paramsDst != 0)
    {
        if(Kp >= 0.0f)
            paramsDst->Kp = Kp;
        if(Ki >= 0.0f)
            paramsDst->Ki = Ki;
    }
}

void PID_WriteParams(PidParams_t* paramsDst, const PidParams_t* paramsSrc)
{
    if(paramsDst != 0 && paramsSrc != 0)
    {
        paramsDst->actual    = paramsSrc->actual;
        paramsDst->setPoint  = paramsSrc->setPoint;
        paramsDst->errorInt  = paramsSrc->errorInt;
        paramsDst->output    = paramsSrc->output;
        paramsDst->rampPoint = paramsSrc->rampPoint;
    }
}

void PID_ClearParams(PidParams_t *paramsDst)
{
    if(paramsDst != 0)
    {
        PidParams_t zeroPID;
        zeroPID.actual     = 0.0f;
        zeroPID.setPoint   = 0.0f;
        zeroPID.errorInt   = 0.0f;
        zeroPID.output     = 0.0f;
        zeroPID.rampPoint  = 0.0f;
        zeroPID.Ki         = paramsDst->Ki; // keep the Ki value
        zeroPID.Kp         = paramsDst->Kp; // keep the Kp value

        PID_WriteParams(paramsDst, (const PidParams_t*)&zeroPID);
    }
}
