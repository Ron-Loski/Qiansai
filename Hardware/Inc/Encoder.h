#ifndef __ENCODEE_H_
#define __ENCODEE_H_


#include "ch32v30x.h"
#include "ch32v30x_rcc.h"
#include "ch32v30x_gpio.h"
#include "ch32v30x_tim.h"

#define ENCODER_1    1
#define ENCODER_2    2
#define wheellen     15          

void Encoder_Init(void);
int16_t Encoder_GetCount(uint8_t Encoder);


#endif
