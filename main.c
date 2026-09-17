#include "stm32f10x.h"                  // Device header
#include "PWM-Motor.h"
#include "Delay.h"

int main(void) {
		static uint8_t  tmp = 0;
		static uint16_t s_tim = 0;
	
    Motor_Init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    Motor_SwitchState(0); // Switch ON motor_L
    GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);

    while (1) {
		if( (tmp++) % 2 )
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
		else
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
				if(s_tim++ > 300)	 break;
		
        Motor_SetSpeed_M0(67.78);
        Motor_SetSpeed_M1(67.78);
        Motor_SetSpeed_M2(67.78);
        Motor_SetSpeed_M3(67.78);
        Delay_ms(10);
    }
		
		
        Motor_SetSpeed_M0((float)0);
        Motor_SetSpeed_M1((float)0);
        Motor_SetSpeed_M2((float)0);
        Motor_SetSpeed_M3((float)0);
}
