#include "stm32f10x.h"                  // Device header
#include "PWM-Motor.h"
#include "Motion.h"
#include "Delay.h"
#include "cmtSys_TIM.h"
#include "LEDs.h"
#include "Keys.h"
#include "HC05.h"
#include "Enc.h"
#include "PID.h"
#include <string.h>

#define __TEMP__ENC_MSG_INTVL_MS    1145

static uint32_t __TEMP__nextEncMsgMs;
static float g_actualRPM[2];                // Encoder module:       used to store the speed collected from encoder in RPM
                                            ///                      @ref `uint8_t Enc_SampleSpeed(uint16_t *speed, uint32_t nowMs)`
static float g_targetRPM[2];                // Motion, PID module:   used to store the target RPM for the motion
                                            ///                      @ref `void Motion_SetMotion(Motion_t motion, float *argv, float *g_targetRPM)`

void CmtSys_Init()
{
    /* Module Init */
    
    Motion_Init();
    LED_Init();
    Key_Init();
    HC05_Init();
    Enc_Init();
    PID_Init();

    // Init Indicator
    for(int i = 0; i < 5; i++)
    {
        LED_TurnON((LEDslot)LED_MAIN);
        Delay_ms(114);
        LED_TurnOFF((LEDslot)LED_MAIN);
        Delay_ms(114);
    }

    CmtSysTim_Init();

    /* Cleanup for startup */
    __TEMP__nextEncMsgMs = 0;
}

void CmtSys_Update()
{
    /* Forward Decl Section */
    
    uint32_t nowMs;                            // cmtSys_TIM module:    system time
    uint8_t keyEvents[KEY_EV_COUNT];           // Keys module:
                                               ///                      @ref @var `g_CurrKeyEV[KEY_EV_COUNT]`
    uint8_t hc05RxFlag;                        // HC05 module:          
                                               ///                      @ref @var `HC05_RxFlag`
    char hc05Msg[HC05_UART_PACKET_MAXLEN];     // HC05 module:          
                                               ///                      @ref @var `HC05_RxPacket`
    float controlledPWM[2];                    // PID module:           used to store the control output from PID_Update()
                                               ///                      @ref `float* PID_Update(float* actualRPM, float* targetRPM)`

    /* `Update()` Section */

    nowMs = CmtSysTim_GetMs();
    Key_GetEvents(keyEvents);
    hc05RxFlag = HC05_GetRxFlag();
    if(Enc_SampleSpeed(g_actualRPM, nowMs) != 0)
    {
        float *value = PID_Update(g_actualRPM, g_targetRPM);
        
        controlledPWM[0] = value[0];
        controlledPWM[1] = value[1];
        Motion_SetSpeed(controlledPWM);
    }

    LED_CheckFlicker(nowMs);
    
    if(__TEMP__nextEncMsgMs < nowMs)
    {
        HC05_SendString("\r\n");
        HC05_SendStringFormatted("PWM:%f,%f.\r\n"   , controlledPWM[0], controlledPWM[1]);
        HC05_SendStringFormatted("actual:%f,%f.\r\n", g_actualRPM[0], g_actualRPM[1]);
        HC05_SendStringFormatted("target:%f,%f.\r\n", g_targetRPM[0], g_targetRPM[1]);
        __TEMP__nextEncMsgMs = nowMs + __TEMP__ENC_MSG_INTVL_MS;
    }

    /* Keys Module Action */
    
    if(keyEvents[KEY_EV_LED_ON])
    {
        LED_SetFlicker(nowMs, (LEDslot)LED_MSG_RX, 3, 114);
    }

    /* RC Module Action */
    
    if(hc05RxFlag != 0U)
    {
        HC05_GetRxPacket(hc05Msg);
        LED_SetFlicker(nowMs, (LEDslot)LED_MSG_RX, 3, 114);

        if (strncmp(hc05Msg, "KP+", 3) == 0)
        {
            float newKP = HC05_ParseMsgFloat(hc05Msg + 3);
            PID_SetGains(newKP, -1.0f);
            HC05_SendStringFormatted("CMD:\"SET+KP\" - Y - KP=%f\r\n", newKP);
        }
        else if (strncmp(hc05Msg, "KI+", 3) == 0)
        {
            float newKI = HC05_ParseMsgFloat(hc05Msg + 3);
            PID_SetGains(-1.0f, newKI);
            HC05_SendStringFormatted("CMD:\"SET+KI\" - Y - KI=%f\r\n", newKI);
        }
        else if (strcmp(hc05Msg, "MOTION+FORWARD") == 0)
        {
            float speed[] = {314.0f, 314.0f};
            Motion_SetMotion((Motion_t)MOTION_FORWARD, speed, g_targetRPM);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+FORWARD\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+FORWARD_LEFT") == 0)
        {
            float speed[] = {114.0f, 314.0f};
            Motion_SetMotion((Motion_t)MOTION_FORWARD, speed, g_targetRPM);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+FORWARD_LEFT\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+FORWARD_RIGHT") == 0)
        {
            float speed[] = {314.0f, 1144.0f};
            Motion_SetMotion((Motion_t)MOTION_FORWARD, speed, g_targetRPM);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+FORWARD_RIGHT\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+BACKWARD") == 0)
        {
            float speed[] = {314.0f, 314.0f};
            Motion_SetMotion((Motion_t)MOTION_BACKWARD, speed, g_targetRPM);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+BACKWARD\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+LEFT") == 0)
        {
            float speed[] = {66.0f, 67.0f};
            Motion_SetMotion((Motion_t)MOTION_LEFT, speed, g_targetRPM);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+LEFT\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+RIGHT") == 0)
        {
            float speed[] = {66.0f, 67.0f};
            Motion_SetMotion((Motion_t)MOTION_RIGHT, speed, g_targetRPM);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+RIGHT\" - Y - Succeeded\r\n");
        }
		else if (strcmp(hc05Msg, "MOTION+STOP") == 0)
        {
            float speed[] = {0.0f, 0.0f};
            Motion_SetMotion((Motion_t)MOTION_FORWARD, speed, g_targetRPM);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+STOP\" - Y - Succeeded\r\n");
        }
        else
        {
            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 6, 114);
            HC05_SendString("CMD:\"UNKNOWN.\" - N - Wtf is it even\r\n");
        }

    }
}
