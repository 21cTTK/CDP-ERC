#ifndef HW_I2C_H
#define HW_I2C_H

#include "stm32f10x.h"
#include <stdint.h>

uint8_t HW_I2C_Init(void);
uint8_t HW_I2C_MasterTx(uint8_t addr7, uint8_t *data, uint16_t len);
uint8_t HW_I2C_MasterRx(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len);
uint8_t HW_I2C_ReadReg1Byte(uint8_t addr7, uint8_t reg, uint8_t *byte);
uint8_t HW_I2C_WriteReg1Byte(uint8_t addr7, uint8_t reg, uint8_t byte);
uint8_t HW_I2C_HasInitialized(void);
void    HW_I2C_UpdateError(uint32_t nowMs);

#endif // HW_I2C_H
