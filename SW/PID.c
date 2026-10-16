#include "stm32f10x.h"                  // Device header
#include "PID.h"
#include <math.h>

void PID_Ramp(PidParams_t* params)
{
    if(params->rampPoint < params->setPoint)
    {//forward ramping
        params->rampPoint += params->rampStep;
        if(params->rampPoint > params->setPoint)
            params->rampPoint = params->setPoint;
    }
    else if(params->rampPoint > params->setPoint)
    {//backward ramping
        params->rampPoint -= params->rampStep;
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
    float intClamp = params->iTermClamp / params->Ki;
    if(fabsf(params->errorInt) > intClamp)
    {
        params->errorInt = intClamp * (params->errorInt > 0 ? 1 : -1);
    }

    params->output = params->Kp * currError +
                     params->Ki * params->errorInt;

    //output clamp control output to [-100, 100]
    if (fabsf(params->output) > params->pTermClamp)
    {
        params->output = params->pTermClamp * (params->output > 0 ? 1 : -1);
    }
}

/* Unified PID Parameter Struct WRITE Services */

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

void PID_WriteConfig(PidParams_t* paramsDst, float rampStep, float iTermClamp, float pTermClamp)
{
    if(paramsDst != 0)
    {
        if(rampStep >= 0.0f)
            paramsDst->rampStep = rampStep;
        if(iTermClamp >= 0.0f)
            paramsDst->iTermClamp = iTermClamp;
        if(pTermClamp >= 0.0f)
            paramsDst->pTermClamp = pTermClamp;
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

void PID_CopyParams(PidParams_t* paramsDst, const PidParams_t* paramsSrc)
{
    if(paramsDst != 0 && paramsSrc != 0)
    {
        PID_WriteParams(paramsDst, paramsSrc);
        PID_WriteGains(paramsDst, paramsSrc->Kp, paramsSrc->Ki);
        PID_WriteConfig(paramsDst, paramsSrc->rampStep, paramsSrc->iTermClamp, paramsSrc->pTermClamp);
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
