#ifndef __PWM_MOTOR_H
#define __PWM_MOTOR_H

void Motor_Init(void);
void Motor_SetSpeed_L(float perc);
void Motor_SetSpeed_R(float perc);
void Motor_SwitchState(uint8_t motor);

#endif
