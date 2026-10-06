#include "stm32f10x.h"                  // Device header
#include "PWM-Motor.h"

#define PID_CLAMP_MIN MOTOR_PWM_DUTY_MIN
#define PID_CLAMP_MAX MOTOR_PWM_DUTY_MAX

#define PID_INITIAL_KP  0.1f
#define PID_INITIAL_KI  0.0f

static float g_Kp;
static float g_Ki;

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
    static float s_errorInt[2] = {0, 0};

    //compute curr error
    float currError[2] = {
        targetRPM[0] - actualRPM[0],
        targetRPM[1] - actualRPM[1]
    };
    
    //integrated error
    s_errorInt[0] += currError[0];
    s_errorInt[1] += currError[1];

    //save prev error for next iteration
    s_prevError[0] = currError[0];
    s_prevError[1] = currError[1];

    //compute control output
    float controlOutPWM[2] = {
        g_Kp * currError[0] + g_Ki * s_errorInt[0],
        g_Kp * currError[1] + g_Ki * s_errorInt[1]
    };

    //clamp control output to [-100, 100]
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
    g_Kp = Kp;
    g_Ki = Ki;
}
