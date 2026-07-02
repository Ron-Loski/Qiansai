#ifndef __ARM_SERVO_H_
#define __ARM_SERVO_H_

#include "ch32v30x.h"
#include <stdint.h>

/* ==================== 舵机编号宏定义 ==================== */
#define ARM_SERVO_J1_BASE        0U    /* J1：底座旋转舵机，对应 PA6 / TIM3_CH1 */
#define ARM_SERVO_J2_SHOULDER    1U    /* J2：肩关节舵机，对应 PA7 / TIM3_CH2 */
#define ARM_SERVO_J3_ELBOW       2U    /* J3：肘关节舵机，对应 PB0 / TIM3_CH3 */
#define ARM_SERVO_J4_WRIST       3U    /* J4：腕部/末端舵机，对应 PB1 / TIM3_CH4 */

/* 当前只给出了 4 个 PWM 引脚，因此顶端抓取舵机默认复用 J4 通道。
 * 如果实际机械臂抓取需要独立第五个舵机，需要再增加一个 PWM 引脚并扩展 ARM_SERVO_COUNT。 */
#define ARM_SERVO_GRIPPER        ARM_SERVO_J4_WRIST
#define ARM_SERVO_COUNT          4U

/* ==================== DS3115 舵机控制参数 ==================== */
#define ARM_SERVO_PWM_FREQ_HZ        50U       /* 舵机常用 50Hz，即 20ms 周期 */
#define ARM_SERVO_PERIOD_US          20000U    /* 20ms = 20000us */
#define ARM_SERVO_MIN_PULSE_US       500U      /* 0 度对应约 500us */
#define ARM_SERVO_MAX_PULSE_US       2500U     /* 180 度对应约 2500us */
#define ARM_SERVO_MIN_ANGLE_DEG      0.0f
#define ARM_SERVO_MAX_ANGLE_DEG      180.0f

/* 软启动参数：每次只动一个舵机，并且角度/脉宽都小步变化，降低电流过冲。 */
#define ARM_SERVO_SOFT_STEP_DEG      1.0f
#define ARM_SERVO_SOFT_DELAY_MS      15U
#define ARM_SERVO_SOFT_PULSE_STEP_US 10U

void ArmServo_Init(void);
void ArmServo_SetAngle(uint8_t servo_id, float angle_deg);
void ArmServo_SoftSetAngle(uint8_t servo_id, float target_angle_deg);
void ArmServo_SoftSetAll(const float target_angle_deg[ARM_SERVO_COUNT]); /* 按 J1->J2->J3->J4 顺序逐个软移动 */
float ArmServo_GetAngle(uint8_t servo_id);
uint16_t ArmServo_AngleToPulse(float angle_deg);

#endif


