#ifndef __PWM_H_
#define __PWM_H_

#include "ch32v30x.h"
#include "Varibles.h"

void PWM_Init(void);
void PWM_Brake(void);
void PWM_EnableChannel(Motor_Numtypedef Motor_Num, uint16_t Compare, Motor_Directtypedef Direct);

#endif
