#ifndef __CALLBACK_H_
#define __CALLBACK_H_

#include "ch32v30x.h"

extern volatile uint8_t g_tim6_update_flag;

void TIM6_UpdateCallBack(void);

#endif
