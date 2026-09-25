#ifndef HW_I2C_H
#define HW_I2C_H

#include "stm32f10x.h"
#include <stdint.h>

void HW_I2C_Init(void);
void HW_I2C_UpdateError(uint32_t nowMs);

#endif // HW_I2C_H
