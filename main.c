#include "stm32f10x.h"                  // Device header
#include "cmtSys.h"
#include "Motion.h"

int main(void) {
	static uint8_t  tmp = 0;
	static uint32_t s_tim = 0;

	CmtSys_Init();
    while (1) {
	if( (tmp++) % 2 )
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
	else
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);

	if(s_tim++ > 377891)	 break;

		CmtSys_Update();
    }

	Motion_SetMotion(MOTION_STOP, (void *)0);
}
