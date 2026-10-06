#include "stm32f10x.h"                  // Device header
#include "PWM-Motor.h"
#include "cmtSys_TIM.h"
#include "PID.h"

#define PID_CLAMP_MIN -MOTOR_PWM_DUTY_MAX
#define PID_CLAMP_MAX MOTOR_PWM_DUTY_MAX
#define PID_INT_CLAMP_MIN -110.f / g_Ki
#define PID_INT_CLAMP_MAX 110.f  / g_Ki

#define PID_INITIAL_KP  0.4f
#define PID_INITIAL_KI  0.03f

/// @brief tolerance below which a target change is treated as "no new command";
/// @note  the ramp writes its buf back into the last target on every stage, so the equality
///        check in the restart test only ever fails by float rounding noise
#define PID_RAMP_EPS        0.001f

/// @brief upper bounds for @ref PID_SetRamp, guarding every stage counter against overflow
#define PID_RAMP_MAX_DT_MS  60000U
#define PID_RAMP_MAX_STAGES 1000U

/// @brief default interval between two consecutive stages [ms]
/// @note  keep this equal to `ENC_SPEED_SAMP_INTVL_MS` (HW/Enc.c) so that exactly one stage
///        is released per PID frame; any other value is fine too, the ramp is timed against
///        the mS system clock and simply skips/waits for frames accordingly
#define PID_RAMP_STEP_MS_DEFAULT 50U

static float g_Kp;
static float g_Ki;

volatile float g_currError;
volatile float g_actualRPM;
volatile float g_targetRPM;
volatile float g_controlledPWM;
volatile float g_Int;

/// @brief target RPM actually fed to the controller, i.e. the buffered (ramped) target
/// @note  exposed next to the g_* telemetry above so a debugger/watch window can plot it
volatile float g_bufferedTargetRPM;

/* ================================ target step ramp ================================
 * A new speed command (HC05 msg) never reaches the PID as a single jump: the target is
 * split into `nStages` equal sub-steps, and one sub-step is released every `dtMs`.
 * The ramp is evaluated inside PID_Update(), so the loop rate of the PID itself (the
 * encoder sampling interval) is what discretises the staircase into PID frames.
 * ================================================================================ */

static uint32_t g_rampStepMs     = PID_RAMP_STEP_MS_DEFAULT;
static uint16_t g_rampStages     = PID_RAMP_DEFAULT_STAGES;
static uint32_t g_rampPendStepMs = 0U;      // staged SetRamp() args, applied at the next tick
static uint16_t g_rampPendStages = 0U;
static uint8_t  g_rampPendValid  = 0U;      // 1 while those args are waiting to be applied

static float    g_rampCmdRPM[2];            // last commanded target, the new-command reference
static float    g_rampLastTargetRPM[2];     // last target actually fed to the PID (a stage value)
static float    g_rampStartRPM[2];          // target the current staircase started from
static float    g_rampDistRPM[2];           // total distance still to cover
static uint32_t g_rampStartMs;              // time the current staircase was (re)started
static uint16_t g_rampStage;                // stages already released
static uint8_t  g_rampActive;               // 1 while a staircase is still moving

void PID_Init(void)
{
    g_Kp = PID_INITIAL_KP;
    g_Ki = PID_INITIAL_KI;

    g_rampStepMs      = PID_RAMP_STEP_MS_DEFAULT;
    g_rampStages      = PID_RAMP_DEFAULT_STAGES;
    g_rampPendStepMs  = 0U;
    g_rampPendStages  = 0U;
    g_rampPendValid   = 0U;

    g_rampCmdRPM[0]        = 0.0f;
    g_rampCmdRPM[1]        = 0.0f;
    g_rampLastTargetRPM[0] = 0.0f;
    g_rampLastTargetRPM[1] = 0.0f;
    g_rampStartRPM[0]      = 0.0f;
    g_rampStartRPM[1]      = 0.0f;
    g_rampDistRPM[0]       = 0.0f;
    g_rampDistRPM[1]       = 0.0f;
    g_rampStartMs          = 0U;
    g_rampStage            = 0U;
    g_rampActive           = 0U;
    g_bufferedTargetRPM    = 0.0f;
}

/// @brief (re)configure the speed step ramp
/// @param dtMs    interval between two consecutive stages [ms]; 0 disables the ramp
/// @param nStages number of stages one new target is split into, >= 1; 1 disables the ramp
/// @note the whole staircase therefore takes (nStages - 1) * dtMs to complete
/// @note 0/1 or oversized arguments do not error out, they simply degrade to "no buffering"
void PID_SetRamp(uint32_t dtMs, uint16_t nStages)
{
    if((dtMs == 0U) || (nStages <= 1U) || (dtMs > PID_RAMP_MAX_DT_MS) || (nStages > PID_RAMP_MAX_STAGES))
    {
        dtMs    = 0U;                   // degenerate cfg: deliver every new target at once
        nStages = 1U;
    }

    // staged, so a command arriving between two PID frames can never tear the cfg apart
    g_rampPendStepMs = dtMs;
    g_rampPendStages = nStages;
    g_rampPendValid  = 1U;                  // a degenerate cfg stores dt = 0, so the
                                            // "is there anything pending" test cannot be
                                            // the value itself, it needs this flag
}

/// @brief advance the target staircase by one PID frame
/// @param targetRPM the commanded target, written by the HC05 handler via Motion_SetMotion()
/// @note a change of *command* restarts the staircase from the target currently in use, so
///       a command landing mid-ramp never snaps the PID away from the speed it already has
static void PID_RampTick(float *targetRPM)
{
    //apply a pending SetRamp() as a whole
    if(g_rampPendValid != 0U)
    {
        g_rampStepMs     = g_rampPendStepMs;
        g_rampStages     = g_rampPendStages;
        g_rampPendStepMs = 0U;
        g_rampPendStages = 0U;
        g_rampPendValid  = 0U;
        g_rampActive     = 0U;          // re-quantise the remaining distance under the new cfg
    }

    //new command? compare against the last *command*, never against the last stage value:
    //were the reference the stage value, stage 0 (which equals the start of the staircase)
    //would look like a fresh command on every frame and restart the ramp endlessly
    if((targetRPM[0] > g_rampCmdRPM[0] + PID_RAMP_EPS) || (targetRPM[0] < g_rampCmdRPM[0] - PID_RAMP_EPS) ||
       (targetRPM[1] > g_rampCmdRPM[1] + PID_RAMP_EPS) || (targetRPM[1] < g_rampCmdRPM[1] - PID_RAMP_EPS))
    {
        g_rampCmdRPM[0]        = targetRPM[0];
        g_rampCmdRPM[1]        = targetRPM[1];
        g_rampStartRPM[0]      = g_rampLastTargetRPM[0];    // take off from the speed in use
        g_rampStartRPM[1]      = g_rampLastTargetRPM[1];
        g_rampDistRPM[0]       = targetRPM[0] - g_rampStartRPM[0];
        g_rampDistRPM[1]       = targetRPM[1] - g_rampStartRPM[1];
        g_rampStartMs          = CmtSysTim_GetMs();
        g_rampStage            = 0U;
        g_rampActive           = 1U;
    }

    //release every stage whose time slot has elapsed (handles missed frames / mS wraparound)
    if(g_rampActive != 0U)
    {
        if(g_rampStepMs == 0U)
        {
            g_rampStage  = g_rampStages;
            g_rampActive = 0U;
        }
        else
        {
            uint32_t elapseMs = CmtSysTim_GetMs() - g_rampStartMs;
            uint16_t reached  = (uint16_t)(elapseMs / g_rampStepMs);

            if(reached >= g_rampStages)
            {
                g_rampStage  = g_rampStages;
                g_rampActive = 0U;
            }
            else if(reached > g_rampStage)
            {
                g_rampStage = reached;
            }
        }
    }

    if(g_rampActive != 0U)
    {
        float fraction = (float)g_rampStage / (float)g_rampStages;

        g_rampLastTargetRPM[0] = g_rampStartRPM[0] + fraction * g_rampDistRPM[0];
        g_rampLastTargetRPM[1] = g_rampStartRPM[1] + fraction * g_rampDistRPM[1];
    }
    else
    {
        // settled: hold whatever was commanded when the staircase finished, not the live
        // command, so a value changed between two frames is picked up by the test above
        g_rampLastTargetRPM[0] = g_rampCmdRPM[0];
        g_rampLastTargetRPM[1] = g_rampCmdRPM[1];
    }

    g_bufferedTargetRPM = g_rampLastTargetRPM[0];
}

/// @brief 
/// @param actualRPM
/// @param targetRPM
/// @note - update interval matches that of @ref ENC_SPEED_SAMP_INTVL_MS 
/// @note - @p targetRPM is the *command* target; the PID works on the buffered staircase
///         target produced by @ref PID_RampTick(), which is called from here
/// @return - control output float array
float* PID_Update(float* actualRPM, float* targetRPM)
{
    static float s_prevError[2] = {0, 0};
    static float s_errorIntRPM[2] = {0, 0};

    //buffer the commanded target into a dt/n step staircase
    PID_RampTick(targetRPM);

    //compute curr error
    float currErrorRPM[2] = {
        g_rampLastTargetRPM[0] - actualRPM[0],
        g_rampLastTargetRPM[1] - actualRPM[1]
    };
    
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
    /// @note `static` is what lets PID_Update() hand this array back by address at all; as a
    ///       plain local it would dangle the moment the function returns, so the caller must
    ///       still consume the returned pointer before the next PID_Update() call
    static float controlOutPWM[2];
    controlOutPWM[0] = g_Kp * currErrorRPM[0] + g_Ki * s_errorIntRPM[0];
    controlOutPWM[1] = g_Kp * currErrorRPM[1] + g_Ki * s_errorIntRPM[1];
    
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
    g_targetRPM = targetRPM[0];                 // commanded target
    g_bufferedTargetRPM = g_rampLastTargetRPM[0];   // buffered target the PID actually used
    g_controlledPWM = controlOutPWM[0];
    
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
