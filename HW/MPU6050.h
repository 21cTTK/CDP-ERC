#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

uint8_t MPU6050_Init(void);
uint8_t MPU6050_ReadYawData(int16_t *data);
void    MPU6050_YawRawToDps(int16_t *raw);
uint8_t MPU6050_IsReady(void);
uint8_t MPU6050_SampleTimeReached(uint32_t nowMs);

#endif
