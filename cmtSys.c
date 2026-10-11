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
    //...
}

/// @brief Main area for module interaction and system activities 
void CmtSys_Update()
{
    /* Intermediate Variables' Forward Decl Section */
    
    uint32_t nowMs;                             //!< cmtSys_TIM module: provides system time
    uint16_t encCnt[2] = {0,0};                 //!< Encoder module: used to store the read-in CNT value. @ref Enc_GetCNT
    uint8_t  keyEvents[KEY_EV_COUNT];           //!< Keys module: @ref g_CurrKeyEV
    uint8_t  hc05RxFlag;                        //!< HC05 module: @ref HC05_RxFlag
    int16_t  yawData;                           //!< Yaw data.
    char     hc05Msg[HC05_UART_PACKET_MAXLEN];  //!< HC05 module: @ref HC05_RxPacket
    float    actualRPM[2];                      //!< PID module: used to store the actual RPM value from @ref Enc_SampleSpeed
    float    targetRPM[2];                      //!< PID module: used to store the target RPM value from @ref Motion_SetTarget
    float    controlledPWM[2];                  //!< PID module: used to store the control output from @ref PID_Update

    /* `Update()` Section */

    nowMs = CmtSysTim_GetMs();
    Key_GetEvents(keyEvents);
    hc05RxFlag = HC05_GetRxFlag();
    
    PID_GetActualRPM(actualRPM);
    PID_GetTargetRPM(targetRPM);
    if(Enc_SampleSpeed(actualRPM, nowMs) != 0)
    {
        float *currStep      = PID_Ramp();
        float *controlOutput = PID_Update(actualRPM, currStep, Motion_IsStopped());
        
        controlledPWM[0] = controlOutput[0];
        controlledPWM[1] = controlOutput[1];
        Motion_SetSpeed(controlledPWM);
    }
    
    if(HC05_PidMsgTimeReached(nowMs) != 0U)
    {
        HC05_SendStringFormatted("%f,%f,%f\r\n", actualRPM[0], targetRPM[0], controlledPWM[0]);
    }

    if(MPU6050_IsReady() != 0U)
    {// yep it can work
        if(MPU6050_SampleTimeReached(nowMs) != 0U)
        {// yep we should sample it
            if(MPU6050_ReadYawData(&yawData) == 0U)
            {// yes! we got it
                MPU6050_YawRawToDps(&yawData);
                HC05_SendStringFormatted("YAW:%d.\r\n", yawData);
            }
            else
            {// no! seems to have failed
                HC05_SendString("YAW:ERROR.\r\n");
            }
        }
        else;//wait until i want it
    }
    else
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
            float speed[] = {+314.0f, +314.0f};
            PID_SetTargetRPM(speed);
            Motion_SetStop(0);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+FORWARD\" - Y - Succeeded\r\n");
        }
        else if (strncmp(hc05Msg, "FORWARD+", 8) == 0)
        {
            float newSpeed = HC05_ParseMsgFloat(hc05Msg + 8);
            float speed[] = {newSpeed, newSpeed};
            PID_SetTargetRPM(speed);
            Motion_SetStop(0);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+FORWARD\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+FORWARD_LEFT") == 0)
        {
            float speed[] = {+114.0f, +314.0f};
            PID_SetTargetRPM(speed);
            Motion_SetStop(0);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+FORWARD_LEFT\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+FORWARD_RIGHT") == 0)
        {
            float speed[] = {+314.0f, +114.0f};
            PID_SetTargetRPM(speed);
            Motion_SetStop(0);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+FORWARD_RIGHT\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+BACKWARD") == 0)
        {
            float speed[] = {-314.0f, -314.0f};
            PID_SetTargetRPM(speed);
            Motion_SetStop(0);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+BACKWARD\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+STOP") == 0)
        {
            PID_SetTargetRPM((float[]){0.0f, 0.0f});
            Motion_SetStop(1);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+STOP\" - Y - Succeeded\r\n");
        }
		else if (strcmp(hc05Msg, "MOTION+PAUSE") == 0)
        {
            float speed[] = {0.0f, 0.0f};
            PID_SetTargetRPM(speed);
            Motion_SetStop(0);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+PAUSE\" - Y - Succeeded\r\n");
        }
        else
        {
            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 6, 114);
            HC05_SendString("CMD:\"UNKNOWN.\" - N - Wtf is it even\r\n");
        }

    }
}
