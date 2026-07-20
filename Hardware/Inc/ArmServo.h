#ifndef __ARM_SERVO_H_
#define __ARM_SERVO_H_

#include "ch32v30x.h"
#include <stdint.h>

/* Four physical servos: three pose joints plus one independent gripper. */
#define ARM_SERVO_J1_BASE        0U    /* PA6 / TIM3_CH1 */
#define ARM_SERVO_J2_SHOULDER    1U    /* PA7 / TIM3_CH2 */
#define ARM_SERVO_J3_WRIST       2U    /* PB0 / TIM3_CH3 */
#define ARM_SERVO_GRIPPER        3U    /* PB1 / TIM3_CH4 */
#define ARM_SERVO_COUNT          4U

#define ARM_SERVO_PWM_FREQ_HZ       50U
#define ARM_SERVO_PERIOD_US       20000U
#define ARM_SERVO_MIN_PULSE_US      500U
#define ARM_SERVO_MAX_PULSE_US     2500U
#define ARM_SERVO_MIN_ANGLE_DEG      0.0f
#define ARM_SERVO_MAX_ANGLE_DEG    180.0f

#define ARM_SERVO_SOFT_STEP_DEG      1.0f
#define ARM_SERVO_SOFT_DELAY_MS     15U
#define ARM_SERVO_ENABLE_DELAY_MS  250U

void ArmServo_Init(void);
void ArmServo_SetInitialAngle(uint8_t servo_id, float angle_deg);
void ArmServo_SetAngle(uint8_t servo_id, float angle_deg);
void ArmServo_SoftSetAngle(uint8_t servo_id, float target_angle_deg);
void ArmServo_SoftSetAll(const float target_angle_deg[ARM_SERVO_COUNT]);
float ArmServo_GetAngle(uint8_t servo_id);
uint8_t ArmServo_IsEnabled(uint8_t servo_id);
uint16_t ArmServo_AngleToPulse(float angle_deg);

/* STOP keeps the current PWM/holding torque and only interrupts soft motion. */
void ArmServo_RequestStop(void);
void ArmServo_ClearStop(void);
uint8_t ArmServo_StopRequested(void);

#endif
