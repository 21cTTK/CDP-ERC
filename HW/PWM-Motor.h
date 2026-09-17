#ifndef __PWM_MOTOR_H
#define __PWM_MOTOR_H

void Motor_Init(void);
void Motor_SetSpeed_M0(float perc);
void Motor_SetSpeed_M1(float perc);
void Motor_SetSpeed_M2(float perc);
void Motor_SetSpeed_M3(float perc);
void Motor_SwitchState(uint8_t motor);

#endif
