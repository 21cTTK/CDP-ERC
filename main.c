#include "stm32f10x.h"                  // Device header
#include "PWM-Motor.h"
#include "Delay.h"

int main(void) {
		static uint8_t  tmp = 0;
		static uint16_t s_tim = 0;
	
    

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
