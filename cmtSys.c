#include "stm32f10x.h"                  // Device header
#include "PWM-Motor.h"
#include "Motion.h"
#include "Delay.h"
#include "cmtSys_TIM.h"
#include "LEDs.h"
#include "Keys.h"
#include "HC05.h"
#include "Enc.h"
#include "MPU6050.h"
#include "HW_I2C.h"
#include "PID.h"
#include <string.h>

volatile float tg_yawReading;

void CmtSys_Init()
{
    /* Module Init */
    /**
     * ## Init/inclusion hierarchy
     * 
     * - Motion.h
     *  - PWM-Motor.h
     * - LED.h
     * - Key.h
     * - HC05.h
     * - Enc.h
     * - MPU6050.h
     *  - HW_I2C.h
     * - cmtSys.h
     */
    
    Motion_Init();
    LED_Init();
    Key_Init();
    HC05_Init();
    Enc_Init();
    MPU6050_Init();

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
    //...
}

/// @brief Main area for module interaction and system activities 
void CmtSys_Update()
{
    /* Intermediate Variables' Forward Decl Section */
    
    uint32_t     nowMs;                             //!< cmtSys_TIM module: provides system time
    uint8_t      keyEvents[KEY_EV_COUNT];           //!< Keys module: @ref g_CurrKeyEV
    uint8_t      hc05RxFlag;                        //!< HC05 module: @ref HC05_RxFlag
    char         hc05Msg[HC05_UART_PACKET_MAXLEN];  //!< HC05 module: @ref HC05_RxPacket
    PidParams_t* speedPID[2];                   //!< PID module: @ref g_speedPID
    PidParams_t* offsetPID;

    /* `Update()` Section */

    nowMs = CmtSysTim_GetMs();
    Key_GetEvents(keyEvents);
    hc05RxFlag = HC05_GetRxFlag();

    if(HC05_PidMsgTimeReached(nowMs) != 0U)
    {
        Enc_GetSpeedPidParams(0, &speedPID[0]);
        Enc_GetSpeedPidParams(1, &speedPID[1]);
        HC05_SendStringFormatted("%f,%f,%f\r\n", speedPID[0]->actual, etg_actualSet, speedPID[0]->output);

        if(MPU6050_IsMsgReady() != 0U)
        {// yes! we got it; the yaw rate now lives in the MPU6050 offset PID's `actual`
            MPU6050_GetOffsetPidParams(&offsetPID);
			tg_yawReading = offsetPID->actual;
            HC05_SendStringFormatted("YAW:%f.\r\n", offsetPID->actual);
        }
    }

    if(MPU6050_IsReady() == 0U)
    {// Re-try initialization
        MPU6050_Init();
    }

    HW_I2C_UpdateError(nowMs);

    /* Keys Module Action */
    
    if(keyEvents[KEY_EV_LED_ON])
    {
        LED_SetFlicker(nowMs, (LEDslot)LED_MSG_RX, 3, 114);
    }

    /* RC Module Action */
    
    if(hc05RxFlag != 0U)
    {
        Enc_GetSpeedPidParams(0, &speedPID[0]);
        Enc_GetSpeedPidParams(1, &speedPID[1]);
        MPU6050_GetOffsetPidParams(&offsetPID);
        HC05_GetRxPacket(hc05Msg);
        LED_SetFlicker(nowMs, (LEDslot)LED_MSG_RX, 3, 114);

        float speedTarget[2] = {speedPID[0]->setPoint, speedPID[1]->setPoint};
        float offsetTarget   = offsetPID->setPoint;

        if (strncmp(hc05Msg, "KP+", 3) == 0)
        {
            float newKP = HC05_ParseMsgFloat(hc05Msg + 3);
            PID_WriteGains(speedPID[0], newKP, -1.0f);
            PID_WriteGains(speedPID[1], newKP, -1.0f);
            HC05_SendStringFormatted("CMD:\"SET+KP\" - Y - KP=%f\r\n", newKP);
        }
        else if (strncmp(hc05Msg, "KI+", 3) == 0)
        {
            float newKI = HC05_ParseMsgFloat(hc05Msg + 3);
            PID_WriteGains(speedPID[0], -1.0f, newKI);
            PID_WriteGains(speedPID[1], -1.0f, newKI);
            HC05_SendStringFormatted("CMD:\"SET+KI\" - Y - KI=%f\r\n", newKI);
        }
        else if (strncmp(hc05Msg, "YAWKP+", 6) == 0)
        {
            float newKP = HC05_ParseMsgFloat(hc05Msg + 6);
            PID_WriteGains(offsetPID, newKP, -1.0f);
            HC05_SendStringFormatted("CMD:\"SET+YAWKP\" - Y - KP=%f\r\n", offsetPID->Kp);
        }
        else if (strncmp(hc05Msg, "YAWKI+", 6) == 0)
        {
            float newKI = HC05_ParseMsgFloat(hc05Msg + 6);
            PID_WriteGains(offsetPID, -1.0f, newKI);
            HC05_SendStringFormatted("CMD:\"SET+YAWKI\" - Y - KI=%f\r\n", offsetPID->Ki);
        }
        else if (strncmp(hc05Msg, "YAWOFFSET+", 10) == 0)
        {
            float newOffset = HC05_ParseMsgFloat(hc05Msg + 10);
            MPU6050_WriteOffset(newOffset);
            HC05_SendStringFormatted("CMD:\"SET+YAWOFFSET\" - Y - OFFSET=%f\r\n", newOffset);
        }
        else if (strcmp(hc05Msg, "MOTION+FORWARD") == 0)
        {
            speedTarget[0] = 240.0f;
            speedTarget[1] = 240.0f;
            offsetTarget   = 0.0f;

            Motion_SetStop(0);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+FORWARD\" - Y - Succeeded\r\n");
        }
        else if (strncmp(hc05Msg, "FORWARD+", 8) == 0)
        {
            float newSpeed = HC05_ParseMsgFloat(hc05Msg + 8);
            speedTarget[0] = newSpeed;
            speedTarget[1] = newSpeed;
            offsetTarget   = 0.0f;

            Motion_SetStop(0);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+FORWARD\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+LEFT") == 0)
        {
            speedTarget[0] = -114.0f;
            speedTarget[1] = 314.0f;
            offsetTarget   = 114.0f;

            Motion_SetStop(0);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+LEFT\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+RIGHT") == 0)
        {
            speedTarget[0] = 114.0f;
            speedTarget[1] = -314.0f;
            offsetTarget   = -114.0f;

            Motion_SetStop(0);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+RIGHT\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+BACKWARD") == 0)
        {
            speedTarget[0] = -240.0f;
            speedTarget[1] = -240.0f;
            offsetTarget   = 0.0f;

            Motion_SetStop(0);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+BACKWARD\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+STOP") == 0)
        {
            speedTarget[0] = 0.0f;
            speedTarget[1] = 0.0f;
            offsetTarget   = 0.0f;

            Motion_SetStop(1);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+STOP\" - Y - Succeeded\r\n");
        }
		else if (strcmp(hc05Msg, "MOTION+PAUSE") == 0)
        {
            speedTarget[0] = 0.0f;
            speedTarget[1] = 0.0f;
            offsetTarget   = 0.0f;

            Motion_SetStop(0);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+PAUSE\" - Y - Succeeded\r\n");
        }
        else
        {
            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 6, 114);
            HC05_SendString("CMD:\"UNKNOWN.\" - N - Wtf is it even\r\n");
        }

        PidParams_t tempPID[2];
        PID_WriteParams(&tempPID[0], speedPID[0]);
        PID_WriteParams(&tempPID[1], speedPID[1]);

        tempPID[0].setPoint = speedTarget[0];
        tempPID[1].setPoint = speedTarget[1];
        PID_WriteParams(speedPID[0], &tempPID[0]);
        PID_WriteParams(speedPID[1], &tempPID[1]);

        //the yaw target MUST land in the offset PID, or the loop keeps regulating yaw to 0
        offsetPID->setPoint = offsetTarget;
    }
}
