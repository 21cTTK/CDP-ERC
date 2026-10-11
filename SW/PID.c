#include "stm32f10x.h"                  // Device header
#include "Motion.h"
#include "PWM-Motor.h"

#define PID_CLAMP_MIN -MOTOR_PWM_DUTY_MAX
#define PID_CLAMP_MAX MOTOR_PWM_DUTY_MAX
#define PID_INT_CLAMP_MIN -110.f / g_Ki
#define PID_INT_CLAMP_MAX 110.f  / g_Ki
#define PID_RAMP_STEP_SIZE 32.0f

#define PID_INITIAL_KP  0.55f
#define PID_INITIAL_KI  0.09f

static float g_Kp;
static float g_Ki;
static float g_actualRPM[2];
static float g_targetRPM[2];

void PID_Init(void)
{
    g_Kp = PID_INITIAL_KP;
    g_Ki = PID_INITIAL_KI;
    g_actualRPM[0] = 0.0f;
    g_targetRPM[0] = 0.0f;
    g_actualRPM[1] = 0.0f;
    g_targetRPM[1] = 0.0f;
}

float* PID_Ramp(void)
{
    static float s_currTargetRPM[2] = {0, 0};

    for(int i = 0; i < 2; i++)
    {
        if(s_currTargetRPM[i] < g_targetRPM[i])
        {//forward ramping
            s_currTargetRPM[i] += PID_RAMP_STEP_SIZE;
            if(s_currTargetRPM[i] > g_targetRPM[i])
                s_currTargetRPM[i] = g_targetRPM[i];
        }
        else if(s_currTargetRPM[i] > g_targetRPM[i])
        {//backward ramping
            s_currTargetRPM[i] -= PID_RAMP_STEP_SIZE;
            if(s_currTargetRPM[i] < g_targetRPM[i])
                s_currTargetRPM[i] = g_targetRPM[i];
        }
    }

    return s_currTargetRPM;
}

/// @brief start an iteration of PID control loop
/// @param actualRPM
/// @param targetRPM
/// @param shouldStop if true, the integral term will be reset to zero
/// @note - update interval matches that of @ref ENC_SPEED_SAMP_INTVL_MS 
/// @return - control output float array
float* PID_Update(float* actualRPM, float* targetRPM, uint8_t shouldStop)
{
    static float s_prevError[2] = {0, 0};
    static float s_errorIntRPM[2] = {0, 0};

    g_actualRPM[0] = actualRPM[0];
    g_actualRPM[1] = actualRPM[1];

    //compute curr error
    float currErrorRPM[2] = {
        targetRPM[0] - actualRPM[0],
        targetRPM[1] - actualRPM[1]
    };

    //if isStop is true, reset the integrated error
    if (shouldStop == 1) {
        s_errorIntRPM[0] = 0.0f;
        s_errorIntRPM[1] = 0.0f;
    }

    //integrated error
    s_errorIntRPM[0] += currErrorRPM[0];
    s_errorIntRPM[1] += currErrorRPM[1];

    //clamp integrated error
    if (s_errorIntRPM[0] > PID_INT_CLAMP_MAX) {
        s_errorIntRPM[0] = PID_INT_CLAMP_MAX;
    } else if (s_errorIntRPM[0] < PID_INT_CLAMP_MIN) {
        s_errorIntRPM[0] = PID_INT_CLAMP_MIN;
    }
    if (s_errorIntRPM[1] > PID_INT_CLAMP_MAX) {
        s_errorIntRPM[1] = PID_INT_CLAMP_MAX;
    } else if (s_errorIntRPM[1] < PID_INT_CLAMP_MIN) {
        s_errorIntRPM[1] = PID_INT_CLAMP_MIN;
    }

//    //save prev error for next iteration
//    s_prevError[0] = currErrorRPM[0];
//    s_prevError[1] = currErrorRPM[1];

    //compute control output
    float controlOutPWM[2] = {
        g_Kp * currErrorRPM[0] + g_Ki * s_errorIntRPM[0],
        g_Kp * currErrorRPM[1] + g_Ki * s_errorIntRPM[1]
    };
    
    //output clamp control output to [-100, 100]
    for(int i = 0; i < 2; i++) {
        if (controlOutPWM[i] > PID_CLAMP_MAX) {
            controlOutPWM[i] = PID_CLAMP_MAX;
        } else if (controlOutPWM[i] < PID_CLAMP_MIN) {
            controlOutPWM[i] = PID_CLAMP_MIN;
        }
    }
    
    //return control output
    return controlOutPWM;
}

void PID_SetGains(float Kp, float Ki)
{
    if(Kp >= 0.0f)
        g_Kp = Kp;
    if(Ki >= 0.0f)
        g_Ki = Ki;
}

void PID_GetActualRPM(float* actualRPM)
{
    actualRPM[0] = g_actualRPM[0];
    actualRPM[1] = g_actualRPM[1];
}

void PID_GetTargetRPM(float* targetRPM)
{
    targetRPM[0] = g_targetRPM[0];
    targetRPM[1] = g_targetRPM[1];
}

void PID_SetTargetRPM(float* targetRPM)
{
    g_targetRPM[0] = targetRPM[0];
    g_targetRPM[1] = targetRPM[1];
}
