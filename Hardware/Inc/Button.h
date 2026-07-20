#ifndef __BUTTON_H_
#define __BUTTON_H_

#include "ch32v30x.h"

#define BUTTON_EVENT_NONE       0x00U
#define BUTTON_EVENT_SINGLE     0x01U
#define BUTTON_EVENT_DOUBLE     0x02U

/* PC2 button, active low. Button_Tick10ms() must be called every 10 ms. */
void Button_Init(void);
void Button_Tick10ms(void);
uint8_t Button_GetEvents(void);
uint8_t Button_PeekEvents(void);
void Button_ClearEvents(void);

#endif
