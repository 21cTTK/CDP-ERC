#include "stm32f10x.h"

/// @brief delay for t_us (us)
/// @param t_us 
void Delay_us(uint32_t t_us)
{
	SysTick->LOAD = 72 * t_us;
	SysTick->VAL = 0x00;
	SysTick->CTRL = 0x00000005;
	while(!(SysTick->CTRL & 0x00010000));
	SysTick->CTRL = 0x00000004;
}

/// @brief delay for t_ms (ms)
/// @param t_ms 
void Delay_ms(uint32_t t_ms)
{
	while(t_ms--)
	{
		Delay_us(1000);
	}
}

/// @brief delay for t_s (s)
/// @param t_s 
void Delay_s(uint32_t t_s)
{
	while(t_s--)
	{
		Delay_ms(1000);
	}
} 
