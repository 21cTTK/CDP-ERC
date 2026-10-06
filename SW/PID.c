#include "stm32f10x.h"                  // Device header
#include "PID.h"
#include "PWM-Motor.h"

/* =========================== Generic PI core ============================= */

/// @brief fill in a P+I controller
/// @param pid the controller to set up
/// @param kp proportional gain
/// @param ki integral gain
/// @param outMin lower output saturation limit
/// @param outMax upper output saturation limit
void PID_Init(PID_t *pid, float kp, float ki, float outMin, float outMax)
{
    pid->kp       = kp;
    pid->ki       = ki;
    pid->outMin   = outMin;
    pid->outMax   = outMax;
    pid->setpoint = 0.0f;
    pid->iTerm    = 0.0f;
    pid->out      = 0.0f;
}

/// @brief drop the accumulated integral and the last output
void PID_Reset(PID_t *pid)
{
    pid->iTerm = 0.0f;
    pid->out   = 0.0f;
}

/// @brief retarget the controller
/// @note the integral is restarted as well, so that a large setpoint step cannot
///       wind it up before the feedback has had a chance to move
void PID_SetSetpoint(PID_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
    PID_Reset(pid);
}

/// @brief run one P+I step
/// @param pid the controller to step
/// @param feedback process value, in the same unit as the setpoint
/// @param dtSec time since the previous step, in seconds
/// @return the saturated output, kept within [`outMin`, `outMax`]
float PID_Update(PID_t *pid, float feedback, float dtSec)
{
    float err   = pid->setpoint - feedback;
    float pTerm = pid->kp * err;
    float iTerm = pid->iTerm + (pid->ki * err * dtSec);
    float out;

    /* anti-windup, part 1: the integral alone must never be able to ask for more
       than the actuator range, whatever the error did in the meantime */
    if(iTerm > pid->outMax)
        iTerm = pid->outMax;
    else if(iTerm < pid->outMin)
        iTerm = pid->outMin;

    out = pTerm + iTerm;

    /* saturate the output */
    if(out > pid->outMax)
        out = pid->outMax;
    else if(out < pid->outMin)
        out = pid->outMin;

    /* anti-windup, part 2: while the output sits on a limit, keep charging the
       integral only when the error is pulling it back off that limit */
    if(!((err > 0.0f && out >= pid->outMax) ||
         (err < 0.0f && out <= pid->outMin)))
        pid->iTerm = iTerm;

    pid->out = out;
    return out;
}

/* ==================== Speed loop over `rawSpeedRPM[2]` =================== */

static PID_t    g_SpeedPID[PID_CHANNEL_COUNT];
static uint32_t g_SpeedLastMs;
static uint8_t  g_SpeedHasLastMs;

/// @brief bring up both wheel speed loops, with the output parked at zero
void PID_Speed_Init(void)
{
    for(uint8_t i = 0; i < PID_CHANNEL_COUNT; i++)
        PID_Init(&g_SpeedPID[i], PID_SPEED_KP, PID_SPEED_KI,
                 PID_SPEED_OUT_MIN, PID_SPEED_OUT_MAX);

    g_SpeedLastMs    = 0;
    g_SpeedHasLastMs = 0;
}

/// @brief retarget both wheel speed loops
/// @param leftPerc target speed of the left wheel, as a duty percentage
/// @param rightPerc target speed of the right wheel, as a duty percentage
/// @note - call this with the same percentages that were handed to
///         `Motion_SetMotion()`, so that a commanded percentage always means the
///         same wheel speed no matter what the battery or the floor does
/// @note - `Motion_SetMotion()` has already pushed those percentages straight to
///         the PWM, so the integrator is seeded with them: the first PI step then
///         continues from the open-loop duty instead of dropping back to zero.
///         The loop itself stays P+I only; with `Kp = Ki = 0` the drive behaves
///         exactly as the open-loop one did
void PID_Speed_SetTarget(float leftPerc, float rightPerc)
{
    float perc[PID_CHANNEL_COUNT] = {leftPerc, rightPerc};

    for(uint8_t i = 0; i < PID_CHANNEL_COUNT; i++)
    {
        if(perc[i] < PID_SPEED_OUT_MIN)
            perc[i] = PID_SPEED_OUT_MIN;
        else if(perc[i] > PID_SPEED_OUT_MAX)
            perc[i] = PID_SPEED_OUT_MAX;

        PID_SetSetpoint(&g_SpeedPID[i], perc[i] * (PID_SPEED_RPM_AT_FULL_DUTY / 100.0f));
        g_SpeedPID[i].iTerm = perc[i];
    }
}

/// @brief run one PI step on each wheel and push the result to the motors
/// @param rawSpeedRPM the encoder feedback, as filled by `Enc_SampleSpeed()`
/// @param nowMs current system time; call this only when a new sample arrived
/// @note - the H-bridge direction is owned by `Motion_t`, so the loops only
///         regulate the wheel speed magnitude and the signed encoder RPM is
///         rectified before it is fed back
void PID_Speed_Update(const float *rawSpeedRPM, uint32_t nowMs)
{
    float dtSec = PID_SPEED_DT_DEFAULT_S;
    float out[PID_CHANNEL_COUNT];

    if(g_SpeedHasLastMs)
    {
        dtSec = (float)(nowMs - g_SpeedLastMs) * 0.001f;
        if(dtSec <= 0.0f)
            dtSec = PID_SPEED_DT_DEFAULT_S;
    }
    g_SpeedLastMs    = nowMs;
    g_SpeedHasLastMs = 1;

    for(uint8_t i = 0; i < PID_CHANNEL_COUNT; i++)
    {
        float feedback = rawSpeedRPM[i];
        if(feedback < 0.0f)
            feedback = -feedback;

        out[i] = PID_Update(&g_SpeedPID[i], feedback, dtSec);
    }

    Motor_L_SetSpeed(out[0]);
    Motor_R_SetSpeed(out[1]);
}
