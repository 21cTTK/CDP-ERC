#include "stm32f10x.h"                  // Device header
#include "PWM-Motor.h"
#include "Motion.h"
#include "Delay.h"
#include "cmtSys_TIM.h"
#include "LEDs.h"
#include "Keys.h"
#include "HC05.h"
#include <string.h>

void CmtSys_Init()
{
    /* Module Init */
    
    Motion_Init();
    LED_Init();
    Key_Init();
    HC05_Init();

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
}

void CmtSys_Update()
{
    /* Forward Decl Section */
    
    uint32_t nowMs;                            // cmtSys_TIM module:    system time
    uint8_t keyEvents[KEY_EV_COUNT];           // Keys module:          @ref `g_CurrKeyEV[KEY_EV_COUNT]`
    uint8_t hc05RxFlag;                        // HC05 module:          @ref `HC05_RxFlag`
    char hc05Msg[HC05_UART_PACKET_MAXLEN];      // HC05 module:          @ref `HC05_RxPacket`

    /* `Update()` Section */

    nowMs = CmtSysTim_GetMs();
    Key_GetEvents(keyEvents);
    hc05RxFlag = HC05_GetRxFlag();

    LED_CheckFlicker(nowMs);

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
            float speed = 67.0f;
            Motion_SetMotion((Motion_t)MOTION_FORWARD, &speed);
            
            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+FORWARD\" - Y - Succeeded\r\n");
        }
        else if (strcmp(hc05Msg, "MOTION+BACKWARD") == 0)
        {
            float speed = 67.0f;
            Motion_SetMotion((Motion_t)MOTION_BACKWARD, &speed);

            LED_SetFlicker(nowMs, (LEDslot)LED_CMD_EXEC, 3, 114);
            HC05_SendString("CMD:\"MOTION+BACKWARD\" - Y - Succeeded\r\n");
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