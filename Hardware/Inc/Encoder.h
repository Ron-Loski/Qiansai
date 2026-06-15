#ifndef _ENCODER_H__
#define _ENCODER_H__

#include "ch32v30x.h"

typedef enum {
	Encoder_Left = 0,
	Encoder_Right
}Encoder_Numtypedef;

void Encoder_Init(void);
int16_t Encoder_Get(Encoder_Numtypedef n);

#endif
