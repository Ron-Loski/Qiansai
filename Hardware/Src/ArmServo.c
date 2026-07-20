#include "ArmServo.h"
#include "debug.h"

static float s_servo_angle[ARM_SERVO_COUNT] = {90.0f, 90.0f, 90.0f, 40.0f};
static uint8_t s_servo_enabled[ARM_SERVO_COUNT] = {0U, 0U, 0U, 0U};
static volatile uint8_t s_stop_requested = 0U;

static float ArmServo_LimitAngle(float angle_deg)
{
    if (angle_deg < ARM_SERVO_MIN_ANGLE_DEG)
    {
        return ARM_SERVO_MIN_ANGLE_DEG;
    }
    if (angle_deg > ARM_SERVO_MAX_ANGLE_DEG)
    {
        return ARM_SERVO_MAX_ANGLE_DEG;
    }
    return angle_deg;
}

uint16_t ArmServo_AngleToPulse(float angle_deg)
{
    float limited_angle = ArmServo_LimitAngle(angle_deg);
    float pulse = (float)ARM_SERVO_MIN_PULSE_US +
                  (limited_angle / 180.0f) *
                      (float)(ARM_SERVO_MAX_PULSE_US - ARM_SERVO_MIN_PULSE_US);
    return (uint16_t)(pulse + 0.5f);
}

static void ArmServo_SetCompare(uint8_t servo_id, uint16_t pulse_us)
{
    switch (servo_id)
    {
    case ARM_SERVO_J1_BASE:
        TIM_SetCompare1(TIM3, pulse_us);
        break;
    case ARM_SERVO_J2_SHOULDER:
        TIM_SetCompare2(TIM3, pulse_us);
        break;
    case ARM_SERVO_J3_WRIST:
        TIM_SetCompare3(TIM3, pulse_us);
        break;
    case ARM_SERVO_GRIPPER:
        TIM_SetCompare4(TIM3, pulse_us);
        break;
    default:
        break;
    }
}

void ArmServo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};
    uint8_t i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Prescaler =
        (uint16_t)(SystemCoreClock / 1000000U - 1U);
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = ARM_SERVO_PERIOD_US - 1U;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0U;

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    for (i = 0U; i < ARM_SERVO_COUNT; i++)
    {
        s_servo_enabled[i] = 0U;
        ArmServo_SetCompare(i, 0U);
    }
    s_stop_requested = 0U;
}

void ArmServo_SetInitialAngle(uint8_t servo_id, float angle_deg)
{
    if (servo_id >= ARM_SERVO_COUNT || s_servo_enabled[servo_id])
    {
        return;
    }
    s_servo_angle[servo_id] = ArmServo_LimitAngle(angle_deg);
}

void ArmServo_SetAngle(uint8_t servo_id, float angle_deg)
{
    if (servo_id >= ARM_SERVO_COUNT)
    {
        return;
    }

    s_servo_angle[servo_id] = ArmServo_LimitAngle(angle_deg);
    ArmServo_SetCompare(servo_id, ArmServo_AngleToPulse(s_servo_angle[servo_id]));
    s_servo_enabled[servo_id] = 1U;
}

void ArmServo_SoftSetAngle(uint8_t servo_id, float target_angle_deg)
{
    float target;
    float current;

    if (servo_id >= ARM_SERVO_COUNT || s_stop_requested)
    {
        return;
    }

    target = ArmServo_LimitAngle(target_angle_deg);
    current = s_servo_angle[servo_id];

    if (!s_servo_enabled[servo_id])
    {
        /* Enable at the configured initial pose; never sweep from a 0 us pulse. */
        ArmServo_SetAngle(servo_id, current);
        Delay_Ms(ARM_SERVO_ENABLE_DELAY_MS);
        if (s_stop_requested)
        {
            return;
        }
    }

    while ((current + ARM_SERVO_SOFT_STEP_DEG) < target ||
           (current - ARM_SERVO_SOFT_STEP_DEG) > target)
    {
        if (s_stop_requested)
        {
            return;
        }

        current += (current < target) ? ARM_SERVO_SOFT_STEP_DEG
                                      : -ARM_SERVO_SOFT_STEP_DEG;
        ArmServo_SetAngle(servo_id, current);
        Delay_Ms(ARM_SERVO_SOFT_DELAY_MS);
    }

    if (!s_stop_requested)
    {
        ArmServo_SetAngle(servo_id, target);
    }
}

void ArmServo_SoftSetAll(const float target_angle_deg[ARM_SERVO_COUNT])
{
    uint8_t i;

    if (target_angle_deg == 0)
    {
        return;
    }

    for (i = 0U; i < ARM_SERVO_COUNT && !s_stop_requested; i++)
    {
        ArmServo_SoftSetAngle(i, target_angle_deg[i]);
        Delay_Ms(ARM_SERVO_SOFT_DELAY_MS);
    }
}

float ArmServo_GetAngle(uint8_t servo_id)
{
    if (servo_id >= ARM_SERVO_COUNT)
    {
        return 0.0f;
    }
    return s_servo_angle[servo_id];
}

uint8_t ArmServo_IsEnabled(uint8_t servo_id)
{
    if (servo_id >= ARM_SERVO_COUNT)
    {
        return 0U;
    }
    return s_servo_enabled[servo_id];
}

void ArmServo_RequestStop(void)
{
    s_stop_requested = 1U;
}

void ArmServo_ClearStop(void)
{
    s_stop_requested = 0U;
}

uint8_t ArmServo_StopRequested(void)
{
    return s_stop_requested;
}
