#include "stm32f10x.h"                  // Device header
#include "cmtSys.h"

int main(void) {
	static uint8_t loopBlink = 0;	// var for Loop Indicator LED; The Loop Indicator should be constantly blinking to indicate system activity

	CmtSys_Init();
    while (1) {
	/* --- BEGIN: Loop Indicator --- */
	if( (loopBlink++) % 2 )
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
	else
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	/* --- END:   Loop Indicator --- */

	/* --- BEGIN: Test area ---*/
	//...
	/* --- END:   Test area --- */

		CmtSys_Update();
    }

	// Indicator for Loop breaked
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
}
