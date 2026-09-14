#include "stm32f10x.h"                  // Device header
#include "PWM-Motor.h"
#include "Delay.h"

int main(void) {
	static uint8_t tmp = 0;
    static uint32_t s_perc = 0;

    Motor_Init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    Motor_SwitchState(0); // Switch ON motor_L

    while (1) {
		if( (tmp++) % 2 )
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
		else
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);

        if(s_perc++ > 100) s_perc = 0;
        Motor_SetSpeed_L((float)s_perc);
        Delay_ms(10);
    }
}
