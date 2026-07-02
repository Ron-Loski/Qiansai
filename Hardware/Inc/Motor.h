#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "ch32v30x.h"
#include "Varibles.h"

#define girth      15.6   // 小车轮子的周长，单位 cm。
#define steplen    20     // 单个地图格子的移动长度，单位 cm。

#define CAR_TURN_ANGLE_THRESHOLD_DEG    3.0f
#define CAR_DEFAULT_ROLL_DEG            0.0f

#define CAR_TURN_PWM_MIN                25U
#define CAR_TURN_PWM_MAX                70U

#define CAR_TURN_PID_KP                 1.2f
#define CAR_TURN_PID_KI                 0.0f
#define CAR_TURN_PID_KD                 0.2f
#define CAR_TURN_PID_INTEGRAL_MAX       100.0f
#define CAR_TURN_PID_OUTPUT_MAX         ((float)CAR_TURN_PWM_MAX)

extern volatile uint8_t stop_flag;

typedef enum
{
    Motor_TurnBusy = 0,
    Motor_TurnOK
} Motor_TurnStatustypedef;

void PWM_Init(void);
void PWM_Brake(void);
void PWM_EnableChannel(Motor_Numtypedef Motor_Num, uint16_t Compare, Motor_Directtypedef Direct);
Motor_TurnStatustypedef car_turn_to_roll(float target_roll);
Motor_TurnStatustypedef car_turn_by_code(uint8_t turn_code, Motor_Directtypedef direct);
void forward_dis(float distance);
void car_move(uint16_t cur_x, uint16_t cur_y, uint16_t minf_x, uint16_t minf_y);
void car_turn(uint8_t a);
void turnround_right(float distance);
void turnround_left(float distance);
void car_return(int8_t par_x, int8_t par_y, int8_t x0, int8_t y0);

#endif