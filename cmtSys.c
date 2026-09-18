#include "stm32f10x.h"                  // Device header
#include "PWM-Motor.h"
#include "Delay.h"
#include "cmtSys_TIM.h"

void CmtSys_Init()
{
    /* Module Init */
    Motor_Init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    CmtSysTim_Init();

    /* Cleanup for startup */
    Motor_SwitchState(0); // Switch ON motor_L
    GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
}

void CmtSys_Update()
{
    uint32_t nowMs = CmtSysTim_GetMs(); 
}