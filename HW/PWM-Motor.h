#ifndef __PWM_MOTOR_H
#define __PWM_MOTOR_H

// Motor Pin Definitions

typedef enum __MOTOR_PIN_DEF_ENUM
{
	AIN1 = 0,
	AIN2,
	BIN1,
	BIN2,
	MOTOR_PIN_COUNT
} Motor_Pin_Def_t;

typedef struct __MOTOR_PIN_STRUCT
{
	GPIO_TypeDef* GPIOx;
	uint16_t 	  GPIO_Pin;
} Motor_Pin_Struct_t;

/// @brief interface/def for TB6612FNG Input Pins
static const Motor_Pin_Struct_t g_MotorPinsDef[MOTOR_PIN_COUNT] =
{
	[AIN1] = {GPIOB, GPIO_Pin_12},
	[AIN2] = {GPIOB, GPIO_Pin_13},
	[BIN1] = {GPIOB, GPIO_Pin_14},
	[BIN2] = {GPIOB, GPIO_Pin_15}
};

void Motor_Init(void);
void Motor_L_SetSpeed(float perc);
void Motor_R_SetSpeed(float perc);
void Motor_SetInput(const uint8_t in[MOTOR_PIN_COUNT]);

#endif//__PWM_MOTOR_H
