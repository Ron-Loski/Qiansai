#ifndef __PWM_H_
#define __PWM_H_

#include "ch32v30x.h"

void PWM_Init(void);
void PWM_Brake(void);
void PWM_EnableChannel(uint8_t Channel, uint16_t Compare);

#endif
