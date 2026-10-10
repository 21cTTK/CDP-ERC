#ifndef __MOTION_H
#define __MOTION_H

typedef enum __MOTION_ENUM
{
	MOTION_STOP = 0,
	MOTION_FORWARD,
	MOTION_BACKWARD,
	MOTION_LEFT,
	MOTION_RIGHT,
	MOTION_COUNT
} Motion_t;

void Motion_Init(void);
void Motion_SetSpeed(float *argv);
void Motion_Stop(void);
uint8_t Motion_IsStopped(void);

#endif//__MOTION_H
