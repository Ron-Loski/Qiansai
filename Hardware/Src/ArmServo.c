#include "ArmServo.h"
#include "debug.h"

static float s_servo_angle[ARM_SERVO_COUNT] = {90.0f, 90.0f, 90.0f, 90.0f};
static uint8_t s_servo_enabled[ARM_SERVO_COUNT] = {0, 0, 0, 0};

/**
 * @brief 限制舵机角度到允许范围。
 * @param angle_deg 输入角度，单位：度。
 * @return 限幅后的角度。
 * @details 使用 ARM_SERVO_MIN_ANGLE_DEG 和 ARM_SERVO_MAX_ANGLE_DEG 防止输出超出普通舵机安全范围。
 */
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

/* 将 0~180 度的舵机角度线性映射为 500~2500us 脉宽。 */
/**
 * @brief 将舵机角度转换为 PWM 脉宽。
 * @param angle_deg 舵机角度，单位：度。
 * @return 对应脉宽，单位：us。
 * @details 将 0~180 度线性映射到 ARM_SERVO_MIN_PULSE_US~ARM_SERVO_MAX_PULSE_US。
 */
uint16_t ArmServo_AngleToPulse(float angle_deg)
{
    float limited_angle = ArmServo_LimitAngle(angle_deg);
    float pulse = (float)ARM_SERVO_MIN_PULSE_US +
                  (limited_angle / 180.0f) * (float)(ARM_SERVO_MAX_PULSE_US - ARM_SERVO_MIN_PULSE_US);
    return (uint16_t)(pulse + 0.5f);
}

/**
 * @brief 写入指定舵机通道的 TIM3 比较值。
 * @param servo_id 舵机编号，0~3 对应 TIM3_CH1~CH4。
 * @param pulse_us PWM 高电平脉宽，单位：us。
 * @return 无。
 * @details 根据 servo_id 选择 TIM_SetCompare1~4，实现 PA6/PA7/PB0/PB1 输出。
 */
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
    case ARM_SERVO_J3_ELBOW:
        TIM_SetCompare3(TIM3, pulse_us);
        break;
    case ARM_SERVO_J4_WRIST:
        TIM_SetCompare4(TIM3, pulse_us);
        break;
    default:
        break;
    }
}

/*
 * PA6/PA7/PB0/PB1 对应 TIM3 的 CH1~CH4，定时器配置为 1MHz 计数频率。
 * 比较值单位等同于 us，ARR=19999 时周期为 20ms，适合普通舵机 PWM。
 */
/**
 * @brief 初始化机械臂舵机 PWM 输出。
 * @param 无。
 * @return 无。
 * @details 配置 TIM3 为 50Hz 舵机 PWM，PA6/PA7/PB0/PB1 为复用推挽输出；初始比较值为 0，避免上电冲击。
 * @note Arm_Init() 会调用本函数，然后通过 ArmServo_SoftSetAll() 软启动到初始角度。
 */
void ArmServo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Prescaler = (uint16_t)(SystemCoreClock / 1000000U - 1U);
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = ARM_SERVO_PERIOD_US - 1U;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    /* 上电初始化时先不输出有效脉宽，避免 4 个舵机同时收到固定占空比造成冲击。 */
    TIM_OCInitStructure.TIM_Pulse = 0;

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

    for (uint8_t i = 0; i < ARM_SERVO_COUNT; i++)
    {
        s_servo_angle[i] = 90.0f;
        s_servo_enabled[i] = 0;
        ArmServo_SetCompare(i, 0);
    }
}

/**
 * @brief 首次启用舵机通道时缓慢增加脉宽。
 * @param servo_id 舵机编号。
 * @param target_angle_deg 目标角度，单位：度。
 * @return 无。
 * @details 从 0us 逐步增加到目标脉宽，降低多个舵机上电瞬间同时动作造成的电流冲击。
 */
static void ArmServo_SoftEnable(uint8_t servo_id, float target_angle_deg)
{
    uint16_t target_pulse;
    uint16_t pulse;

    if (servo_id >= ARM_SERVO_COUNT)
    {
        return;
    }

    if (s_servo_enabled[servo_id])
    {
        return;
    }

    target_pulse = ArmServo_AngleToPulse(target_angle_deg);

    /*
     * 第一次控制某个舵机时，不直接给目标占空比。
     * 先让该通道从 0us 脉宽逐步增加到目标脉宽，减少机械臂上电瞬间的冲击。
     */
    for (pulse = 0; pulse < target_pulse; pulse += ARM_SERVO_SOFT_PULSE_STEP_US)
    {
        ArmServo_SetCompare(servo_id, pulse);
        Delay_Ms(ARM_SERVO_SOFT_DELAY_MS);
    }

    ArmServo_SetCompare(servo_id, target_pulse);
    s_servo_angle[servo_id] = ArmServo_LimitAngle(target_angle_deg);
    s_servo_enabled[servo_id] = 1;
    Delay_Ms(ARM_SERVO_SOFT_DELAY_MS);
}

/**
 * @brief 立即设置单个舵机角度。
 * @param servo_id 舵机编号，0~3。
 * @param angle_deg 目标角度，单位：度。
 * @return 无。
 * @details 角度限幅后直接换算为 PWM 脉宽并写入 TIM3 比较寄存器。
 */
void ArmServo_SetAngle(uint8_t servo_id, float angle_deg)
{
    if (servo_id >= ARM_SERVO_COUNT)
    {
        return;
    }

    s_servo_angle[servo_id] = ArmServo_LimitAngle(angle_deg);
    ArmServo_SetCompare(servo_id, ArmServo_AngleToPulse(s_servo_angle[servo_id]));
}

/**
 * @brief 平滑移动单个舵机到目标角度。
 * @param servo_id 舵机编号，0~3。
 * @param target_angle_deg 目标角度，单位：度。
 * @return 无。
 * @details 先确保该通道软启动，然后按 ARM_SERVO_SOFT_STEP_DEG 分步接近目标角，降低机械冲击。
 */
void ArmServo_SoftSetAngle(uint8_t servo_id, float target_angle_deg)
{
    if (servo_id >= ARM_SERVO_COUNT)
    {
        return;
    }

    float target = ArmServo_LimitAngle(target_angle_deg);
    float current;

    /* 如果舵机还没有输出过有效 PWM，先执行软启动。 */
    ArmServo_SoftEnable(servo_id, target);

    current = s_servo_angle[servo_id];

    while ((current + ARM_SERVO_SOFT_STEP_DEG) < target || (current - ARM_SERVO_SOFT_STEP_DEG) > target)
    {
        if (current < target)
        {
            current += ARM_SERVO_SOFT_STEP_DEG;
        }
        else
        {
            current -= ARM_SERVO_SOFT_STEP_DEG;
        }
        ArmServo_SetAngle(servo_id, current);
        Delay_Ms(ARM_SERVO_SOFT_DELAY_MS);
    }

    ArmServo_SetAngle(servo_id, target);
}

/**
 * @brief 按顺序平滑设置所有机械臂舵机角度。
 * @param target_angle_deg 长度为 ARM_SERVO_COUNT 的目标角数组。
 * @return 无。
 * @details 按 J1->J2->J3->J4 顺序调用 ArmServo_SoftSetAngle()，避免多个舵机同时堵转导致电流过大。
 */
void ArmServo_SoftSetAll(const float target_angle_deg[ARM_SERVO_COUNT])
{
    /*
     * 不要让目标数组维持姿态后让舵机同时运动。
     * 这里按 J1->J2->J3->J4 顺序平滑移动，可降低同时转动造成的电流冲击。
     */
    for (uint8_t i = 0; i < ARM_SERVO_COUNT; i++)
    {
        ArmServo_SoftSetAngle(i, target_angle_deg[i]);
        Delay_Ms(ARM_SERVO_SOFT_DELAY_MS);
    }
}

/**
 * @brief 获取软件记录的舵机当前角度。
 * @param servo_id 舵机编号。
 * @return 当前记录角度；编号无效时返回 0。
 * @details 返回 s_servo_angle 中保存的角度值，不直接读取舵机真实反馈。
 */
float ArmServo_GetAngle(uint8_t servo_id)
{
    if (servo_id >= ARM_SERVO_COUNT)
    {
        return 0.0f;
    }
    return s_servo_angle[servo_id];
}