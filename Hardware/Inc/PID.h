#ifndef __PID_H_
#define __PID_H_

#include "ch32v30x.h"

typedef struct
{
    float Target;
    float Actual;
    float Out;

    float Kp;
    float Ki;
    float Kd;

    float Error0;
    float Error1;
    float ErrorInt;

    float IntegralMax;
    float OutputMax;
} PID_t;

void PID_Init(PID_t *p);
void PID_Update(PID_t *p);

#endif
