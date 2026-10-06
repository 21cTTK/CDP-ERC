#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/* =============================== Tunables ================================ */
/* The only section that has to be adjusted for a given robot.               */

/// @brief number of closed-loop channels: `[0]` = left wheel, `[1]` = right wheel
#define PID_CHANNEL_COUNT           2

/// @brief wheel RPM reached at 100% duty (PWM `CCR = 1000`)
/// @note  this is the one scale that turns the duty percentages used by the
///        `Motion_t` commands into the RPM unit of the encoder feedback.
///        CALIBRATION: send `MOTION+FORWARD` and read the steady `ENC OUTPUT`
///        RPM printed over HC05, then
///        `PID_SPEED_RPM_AT_FULL_DUTY = rpm / (commanded_percentage / 100)`.
#define PID_SPEED_RPM_AT_FULL_DUTY  2500.0f

/// @brief proportional gain, in `[% duty]` per `[RPM]` of error
#define PID_SPEED_KP                0.30f

/// @brief integral gain, in `[% duty]` per `[RPM * s]` of accumulated error
/// @note  the encoder is only sampled every 520 ms, so keep `Ki` small: it moves
///        the integrator by `Ki * error * 0.52` on every sample, and a large
///        value makes the loop oscillate at the sampling rate.
#define PID_SPEED_KI                0.35f

/// @brief output (duty percentage) saturation limits
#define PID_SPEED_OUT_MIN           0.0f
#define PID_SPEED_OUT_MAX           100.0f

/// @brief fallback loop period in seconds; only used for the very first step
/// @note  mirrors `ENC_SPEED_SAMP_INTVL_MS` in `HW/Enc.c`
#define PID_SPEED_DT_DEFAULT_S      0.52f

/* =========================== Generic PI core ============================= */

/// @brief a plain P+I controller; the meaning of its units is left to the caller
typedef struct __PID_STRUCT
{
    float kp;        ///< proportional gain
    float ki;        ///< integral gain
    float setpoint;  ///< target value, in feedback unit
    float iTerm;     ///< accumulated integral contribution, already scaled by `ki`
    float outMin;    ///< lower output saturation limit
    float outMax;    ///< upper output saturation limit
    float out;       ///< output of the last step
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float outMin, float outMax);
void  PID_Reset(PID_t *pid);
void  PID_SetSetpoint(PID_t *pid, float setpoint);
float PID_Update(PID_t *pid, float feedback, float dtSec);

/* ================== Speed loop over `rawSpeedRPM[2]` ==================== */
/* `PID_Speed_Update()` drives the PWM itself, through `Motor_L_SetSpeed()`   */
/* and `Motor_R_SetSpeed()`, so `cmtSys.c` only has to feed it.              */
/* Call `PID_Speed_SetTarget()` with the same percentages that went to        */
/* `Motion_SetMotion()`: the duty it applied is taken as the starting effort  */
/* of the integrator, so the loop trims from the open-loop value.            */

void PID_Speed_Init(void);
void PID_Speed_SetTarget(float leftPerc, float rightPerc);
void PID_Speed_Update(const float *rawSpeedRPM, uint32_t nowMs);

#endif//__PID_H
