#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "ch32v30x.h"
#include "Varibles.h"

#define girth      15.6   //小车轮子的周长(cm)
#define steplen    20     //一格的长度（cm)
extern volatile uint8_t stop_flag;

void PWM_Init(void);
void PWM_Brake(void);
void PWM_EnableChannel(Motor_Numtypedef Motor_Num, uint16_t Compare, Motor_Directtypedef Direct);
uint8_t forward_dis(float distance);
uint8_t car_move(uint16_t cur_x, uint16_t cur_y, uint16_t minf_x, uint16_t minf_y);
uint8_t car_turn(uint8_t direction, float scan_base_yaw);
uint8_t turnround_right(float distance);
uint8_t turnround_left(float distance);   
uint8_t Motor_TurnToAbsoluteYaw(float target_yaw);
uint8_t Motor_TurnToYawCommand(uint8_t command);
uint8_t car_return(int8_t par_x, int8_t par_y, int8_t x0, int8_t y0);

#endif
