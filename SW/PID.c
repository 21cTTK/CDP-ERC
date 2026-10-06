#include "stm32f10x.h"                  // Device header
#include "PWM-Motor.h"

#define PID_CLAMP_MIN MOTOR_PWM_DUTY_MIN
#define PID_CLAMP_MAX MOTOR_PWM_DUTY_MAX

#define PID_KP  0.01f
#define PID_KI  0.001f

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
        PID_KP * currError[0] + PID_KI * s_errorInt[0],
        PID_KP * currError[1] + PID_KI * s_errorInt[1]
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
