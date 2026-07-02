#ifndef __USART_H_
#define __USART_H_

extern uint8_t serial_flag;
void USART2_Init(void);
void Serial_SendString(char *String);
void USART2_IRQHandler(void);
extern uint8_t isobstacle_flag;
extern uint8_t ch;


#endif
