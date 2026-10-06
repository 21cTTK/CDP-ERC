#include "stm32f10x.h"                  // Device header
#include "PWM-Motor.h"

#define PID_CLAMP_MIN -MOTOR_PWM_DUTY_MAX
#define PID_CLAMP_MAX MOTOR_PWM_DUTY_MAX
#define PID_INT_CLAMP_MIN -110.f / g_Ki
#define PID_INT_CLAMP_MAX 110.f  / g_Ki

#define PID_INITIAL_KP  0.4f
#define PID_INITIAL_KI  0.03f

static float g_Kp;
static float g_Ki;

volatile float g_currError;
volatile float g_actualRPM;
volatile float g_targetRPM;
volatile float g_controlledPWM;
volatile float g_Int;

void PID_Init(void)
{
    g_Kp = PID_INITIAL_KP;
    g_Ki = PID_INITIAL_KI;
}

/// @brief 
/// @param actualRPM
/// @param targetRPM
/// @note - update interval matches that of @ref ENC_SPEED_SAMP_INTVL_MS 
/// @return - control output float array
float* PID_Update(float* actualRPM, float* targetRPM)
{
    static float s_prevError[2] = {0, 0};
    static float s_errorIntRPM[2] = {0, 0};

    //compute curr error
    float currErrorRPM[2] = {
        targetRPM[0] - actualRPM[0],
        targetRPM[1] - actualRPM[1]
    };
    
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

    //save prev error for next iteration
    s_prevError[0] = currErrorRPM[0];
    s_prevError[1] = currErrorRPM[1];

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

	g_Int = s_errorIntRPM[0];
    g_currError = currErrorRPM[0];
    g_actualRPM = actualRPM[0];
    g_targetRPM = targetRPM[0];
    g_controlledPWM = controlOutPWM[0];
    
    //return control output
    return controlOutPWM;
}

void PID_SetGains(float Kp, float Ki)
{
    if(Kp > 0.0f)
        g_Kp = Kp;
    if(Ki > 0.0f)
        g_Ki = Ki;
}
