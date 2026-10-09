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
#include <string.h>

#define __TEMP__ENC_MSG_INTVL_MS    1145

static uint32_t __TEMP__nextEncMsgMs;

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
    __TEMP__nextEncMsgMs = 0;
}

void CmtSys_Update()
{
    /* Forward Decl Section */
    
    uint32_t nowMs;                             //!< cmtSys_TIM module: provides system time
    uint16_t encCnt[2] = {0,0};                 //!< Encoder module: used to store the read-in CNT value. @ref Enc_GetCNT
    uint8_t  keyEvents[KEY_EV_COUNT];           //!< Keys module: @ref g_CurrKeyEV
    uint8_t  hc05RxFlag;                        //!< HC05 module: @ref HC05_RxFlag
    int16_t  yawData;                           //!< Yaw data.
    char     hc05Msg[HC05_UART_PACKET_MAXLEN];  //!< HC05 module: @ref HC05_RxPacket


    /* `Update()` Section */

    nowMs = CmtSysTim_GetMs();
    Key_GetEvents(keyEvents);
    hc05RxFlag = HC05_GetRxFlag();

    LED_CheckFlicker(nowMs);
    
    if(__TEMP__nextEncMsgMs < nowMs)
    {
		Enc_GetCNT(encCnt);
        HC05_SendStringFormatted("ENC:%d,%d.\r\n", encCnt[0], encCnt[1]);
        __TEMP__nextEncMsgMs = nowMs + __TEMP__ENC_MSG_INTVL_MS;
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

        if (strcmp(hc05Msg, "MOTION+FORWARD") == 0)
        {
            float speed[] = {67.0f, 67.0f};
            Motion_SetMotion((Motion_t)MOTION_FORWARD, speed);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+FORWARD\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+FORWARD_LEFT") == 0)
        {
            float speed[] = {24.0f, 67.0f};
            Motion_SetMotion((Motion_t)MOTION_FORWARD, speed);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+FORWARD_LEFT\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+FORWARD_RIGHT") == 0)
        {
            float speed[] = {67.0f, 24.0f};
            Motion_SetMotion((Motion_t)MOTION_FORWARD, speed);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+FORWARD_RIGHT\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+BACKWARD") == 0)
        {
            float speed[] = {67.0f, 67.0f};
            Motion_SetMotion((Motion_t)MOTION_BACKWARD, speed);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+BACKWARD\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+LEFT") == 0)
        {
            float speed[] = {66.0f, 67.0f};
            Motion_SetMotion((Motion_t)MOTION_LEFT, speed);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+LEFT\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+RIGHT") == 0)
        {
            float speed[] = {66.0f, 67.0f};
            Motion_SetMotion((Motion_t)MOTION_RIGHT, speed);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+RIGHT\" - Y - Succeeded\r\n");
        }
		else if (strcmp(hc05Msg, "MOTION+STOP") == 0)
        {
            Motion_SetMotion((Motion_t)MOTION_STOP, (float *)NULL);

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
