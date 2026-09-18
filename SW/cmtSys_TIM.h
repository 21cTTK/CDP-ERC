#ifndef __CMTSYS_TIM_H
#define __CMTSYS_TIM_H

void CmtSysTim_Init(void);
uint32_t CmtSysTim_GetMs(void);

void CmtSysTim_ISR(void);

#endif//__CMTSYS_TIM_H
