#include "stm32f10x.h"                  // Device header
#include "PWM-Motor.h"
#include "Motion.h"
#include "Delay.h"
#include "cmtSys_TIM.h"

void CmtSys_Init()
{
    /* Module Init */
    Motion_Init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);

    for(int i = 0; i < 5; i++)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
        Delay_ms(114);
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        Delay_ms(114);
    }

    CmtSysTim_Init();

    /* Cleanup for startup */
}

void CmtSys_Update()
{
    uint32_t nowMs = CmtSysTim_GetMs();

    float speed = 67.91;
    Motion_SetMotion(MOTION_FORWARD, &speed);
}