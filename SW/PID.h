#ifndef __PID_H
#define __PID_H

#include <stdint.h>

float* PID_Update(float* actualRPM, float* targetRPM);

#endif//__PID_H
