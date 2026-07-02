#include "ArmKinematics.h"
#include "debug.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifndef ARM_PI
#define ARM_PI 3.14159265358979323846f
#endif

static volatile char s_host_rx_buf[ARM_HOST_RX_BUF_SIZE];
static volatile uint8_t s_host_rx_index = 0;
static volatile uint8_t s_host_cmd_ready = 0;
static char s_host_cmd_buf[ARM_HOST_RX_BUF_SIZE];
static ArmJointAngles s_current_angles = {ARM_J1_HOME_DEG, ARM_J2_HOME_DEG, ARM_J3_HOME_DEG, ARM_J4_HOME_DEG};

/**
 * @brief 角度转弧度。
 * @param deg 角度值，单位：度。
 * @return 弧度值。
 */static float Arm_DegToRad(float deg)
{
    return deg * ARM_PI / 180.0f;
}

/**
 * @brief 弧度转角度。
 * @param rad 弧度值。
 * @return 角度值，单位：度。
 */static float Arm_RadToDeg(float rad)
{
    return rad * 180.0f / ARM_PI;
}

/**
 * @brief 浮点数上下限约束。
 * @param value 输入值。
 * @param min_value 下限。
 * @param max_value 上限。
 * @return 限幅后的值。
 */static float Arm_LimitFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

/**
 * @brief 检查机械臂四个关节角是否在允许范围内。
 * @param angles 关节角结构体指针。
 * @return 1 表示全部合法，0 表示至少一个关节超限。
 * @details Arm_InverseKinematics() 和 Arm_SetJointAngles() 都调用本函数作为安全保护。
 */static uint8_t Arm_CheckJointRange(const ArmJointAngles *angles)
{
    if (angles->j1 < ARM_J1_MIN_DEG || angles->j1 > ARM_J1_MAX_DEG) return 0;
    if (angles->j2 < ARM_J2_MIN_DEG || angles->j2 > ARM_J2_MAX_DEG) return 0;
    if (angles->j3 < ARM_J3_MIN_DEG || angles->j3 > ARM_J3_MAX_DEG) return 0;
    if (angles->j4 < ARM_J4_MIN_DEG || angles->j4 > ARM_J4_MAX_DEG) return 0;
    return 1;
}

/**
 * @brief 从上位机命令字符串中解析一个整数数值。
 * @param cursor 字符串游标地址，函数会跳过空格/逗号/Tab 并移动到数字后方。
 * @return 解析出的数值，以 float 返回。
 * @note 当前内部使用 strtol()，实际只支持整数坐标，小数不会被正确解析。
 */static float Arm_ParseNumber(char **cursor)
{
    long value;

    while (**cursor == ' ' || **cursor == ',' || **cursor == '\t')
    {
        (*cursor)++;
    }

    value = strtol(*cursor, cursor, 10);
    return (float)value;
}

/*
 * 初始化机械臂，使五轴处于同一工作平面内。
 * 当前只有 4 路 PWM，J4 同时作为末端姿态/抓取通道；若增加第五路 PWM，可将抓取舵机独立出去。
 */
/**
 * @brief 初始化机械臂舵机并回到初始姿态。
 * @param 无。
 * @return 无。
 * @details 调用 ArmServo_Init() 初始化 PWM，再通过 ArmServo_SoftSetAll() 将 J1~J4 移动到 HOME 角度。
 */
void Arm_Init(void)
{
    float home[ARM_SERVO_COUNT] = {ARM_J1_HOME_DEG, ARM_J2_HOME_DEG, ARM_J3_HOME_DEG, ARM_J4_HOME_DEG};

    ArmServo_Init();
    ArmServo_SoftSetAll(home);

    s_current_angles.j1 = ARM_J1_HOME_DEG;
    s_current_angles.j2 = ARM_J2_HOME_DEG;
    s_current_angles.j3 = ARM_J3_HOME_DEG;
    s_current_angles.j4 = ARM_J4_HOME_DEG;

    printf("Arm init finish\r\n");
}

/*
 * 根据图中公式做空间逆解：
 * 1. J1 由目标点在 XY 平面的方向决定。
 * 2. J2/J3/J4 在同一竖直平面内求解。
 * 3. alpha = J2 + J3 + J4，用 ARM_TOOL_PITCH_DEG 固定末端姿态。
 */
/**
 * @brief 根据目标空间坐标计算机械臂四个关节角。
 * @param x/y/z 目标末端坐标，单位：mm。
 * @param angles 输出关节角结构体指针。
 * @return 1 表示逆解成功且关节未超限，0 表示目标不可达或关节超限。
 * @details 先根据 x/y 求 J1，再扣除末端 A4 得到腕点坐标，使用余弦定理解 J2/J3，并由固定末端姿态求 J4。
 * @note Arm_MoveToXYZ() 调用本函数后再调用 Arm_SetJointAngles() 输出到舵机。
 */uint8_t Arm_InverseKinematics(float x, float y, float z, ArmJointAngles *angles)
{
    float planar_len;
    float alpha_rad;
    float wrist_l;
    float wrist_h;
    float cos_j3;
    float sin_j3;
    float j3_rad;
    float k1;
    float k2;
    float sin_j2;
    float cos_j2;
    float j2_rad;
    float j4_deg;

    if (angles == 0)
    {
        return 0;
    }

    planar_len = sqrtf(x * x + y * y) + ARM_BASE_OFFSET_P_MM;
    alpha_rad = Arm_DegToRad(ARM_TOOL_PITCH_DEG);

    /* 扣除末端 A4 后，得到腕关节需要到达的二维坐标 L/H。 */
    wrist_l = planar_len - ARM_LINK_A4_TOOL_MM * sinf(alpha_rad);
    wrist_h = z - ARM_LINK_A1_BASE_HEIGHT_MM - ARM_LINK_A4_TOOL_MM * cosf(alpha_rad);

    cos_j3 = (wrist_l * wrist_l + wrist_h * wrist_h - ARM_LINK_A2_SHOULDER_MM * ARM_LINK_A2_SHOULDER_MM -
              ARM_LINK_A3_ELBOW_MM * ARM_LINK_A3_ELBOW_MM) /
             (2.0f * ARM_LINK_A2_SHOULDER_MM * ARM_LINK_A3_ELBOW_MM);

    if (cos_j3 < -1.0f || cos_j3 > 1.0f)
    {
        printf("Arm IK fail: target out of range\r\n");
        return 0;
    }

    cos_j3 = Arm_LimitFloat(cos_j3, -1.0f, 1.0f);
    sin_j3 = sqrtf(1.0f - cos_j3 * cos_j3);
    j3_rad = atan2f(sin_j3, cos_j3);

    k1 = ARM_LINK_A2_SHOULDER_MM + ARM_LINK_A3_ELBOW_MM * cos_j3;
    k2 = ARM_LINK_A3_ELBOW_MM * sin_j3;

    /* 图中 J2 是相对竖直方向的角度：L=K1*sinJ2+K2*cosJ2，H=K1*cosJ2-K2*sinJ2。 */
    sin_j2 = (k1 * wrist_l - k2 * wrist_h) / (k1 * k1 + k2 * k2);
    cos_j2 = (k1 * wrist_h + k2 * wrist_l) / (k1 * k1 + k2 * k2);
    sin_j2 = Arm_LimitFloat(sin_j2, -1.0f, 1.0f);
    cos_j2 = Arm_LimitFloat(cos_j2, -1.0f, 1.0f);
    j2_rad = atan2f(sin_j2, cos_j2);

    angles->j1 = Arm_RadToDeg(atan2f(y, x));
    if (angles->j1 < 0.0f)
    {
        angles->j1 += 180.0f;
    }

    angles->j2 = Arm_RadToDeg(j2_rad);
    angles->j3 = Arm_RadToDeg(j3_rad);
    j4_deg = ARM_TOOL_PITCH_DEG - angles->j2 - angles->j3;
    angles->j4 = j4_deg;

    if (!Arm_CheckJointRange(angles))
    {
        printf("Arm IK fail: joint limit j1=%.1f j2=%.1f j3=%.1f j4=%.1f\r\n",
               angles->j1, angles->j2, angles->j3, angles->j4);
        return 0;
    }

    return 1;
}

/* 独立的关节角度控制函数，逆解和手动控制最终都走这里。 */
/**
 * @brief 将关节角输出到机械臂舵机。
 * @param angles 目标 J1~J4 角度。
 * @return 1 表示设置成功，0 表示参数为空或关节超限。
 * @details 将角度数组映射到 ARM_SERVO_J1_BASE~ARM_SERVO_J4_WRIST，再调用 ArmServo_SoftSetAll() 平滑动作。
 */uint8_t Arm_SetJointAngles(const ArmJointAngles *angles)
{
    float target[ARM_SERVO_COUNT];

    if (angles == 0 || !Arm_CheckJointRange(angles))
    {
        return 0;
    }

    target[ARM_SERVO_J1_BASE] = angles->j1;
    target[ARM_SERVO_J2_SHOULDER] = angles->j2;
    target[ARM_SERVO_J3_ELBOW] = angles->j3;
    target[ARM_SERVO_J4_WRIST] = angles->j4;

    ArmServo_SoftSetAll(target);
    s_current_angles = *angles;

    printf("Arm move j1=%.1f j2=%.1f j3=%.1f j4=%.1f\r\n",
           angles->j1, angles->j2, angles->j3, angles->j4);
    return 1;
}

/**
 * @brief 控制机械臂末端移动到指定三维坐标。
 * @param x/y/z 目标坐标，单位：mm。
 * @return 1 表示逆解并执行成功，0 表示目标不可达或设置失败。
 * @details 是上位机 P/G 坐标命令最终调用的核心运动函数。
 */uint8_t Arm_MoveToXYZ(float x, float y, float z)
{
    ArmJointAngles angles;

    if (!Arm_InverseKinematics(x, y, z, &angles))
    {
        return 0;
    }

    return Arm_SetJointAngles(&angles);
}

/* 抓取函数独立出来，方便上位机单独发送 OPEN/CLOSE。 */
/**
 * @brief 打开机械臂夹爪。
 * @param 无。
 * @return 无。
 * @details 当前夹爪复用 J4 PWM 通道，调用 ArmServo_SoftSetAngle() 输出 ARM_GRIP_OPEN_DEG。
 */
void Arm_GripOpen(void)
{
    ArmServo_SoftSetAngle(ARM_SERVO_GRIPPER, ARM_GRIP_OPEN_DEG);
    s_current_angles.j4 = ARM_GRIP_OPEN_DEG;
    printf("Grip open\r\n");
}

/**
 * @brief 闭合机械臂夹爪。
 * @param 无。
 * @return 无。
 * @details 当前夹爪复用 J4 PWM 通道，调用 ArmServo_SoftSetAngle() 输出 ARM_GRIP_CLOSE_DEG。
 */
void Arm_GripClose(void)
{
    ArmServo_SoftSetAngle(ARM_SERVO_GRIPPER, ARM_GRIP_CLOSE_DEG);
    s_current_angles.j4 = ARM_GRIP_CLOSE_DEG;
    printf("Grip close\r\n");
}

/* USART1 同时用于 printf TX 和上位机 RX：PA9=TX，PA10=RX。 */
/**
 * @brief 初始化机械臂上位机串口 USART1。
 * @param baudrate 波特率，例如 115200。
 * @return 无。
 * @details PA9 配置为 TX，PA10 配置为 RX，使能 RXNE 中断；USART1 同时用于 printf 输出和接收上位机命令。
 */
void Arm_HostUartInit(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

/**
 * @brief USART1 接收中断服务函数。
 * @param 无。
 * @return 无。
 * @details 按字符接收上位机文本命令，遇到 \r 或 \n 时结束一帧并置位 s_host_cmd_ready。
 */
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        char ch = (char)USART_ReceiveData(USART1);

        if (ch == '\r' || ch == '\n')
        {
            if (s_host_rx_index > 0)
            {
                s_host_rx_buf[s_host_rx_index] = '\0';
                s_host_cmd_ready = 1;
            }
            s_host_rx_index = 0;
        }
        else if (s_host_rx_index < (ARM_HOST_RX_BUF_SIZE - 1U))
        {
            s_host_rx_buf[s_host_rx_index++] = ch;
        }
        else
        {
            s_host_rx_index = 0;
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

/*
 * 上位机命令格式：
 *   P x y z      只移动到指定三维坐标，单位 mm，例如：P 120 0 80
 *   G x y z      移动到指定坐标后执行抓取闭合，例如：G 120 0 80
 *   OPEN         打开抓取
 *   CLOSE        闭合抓取
 */
/**
 * @brief 解析并执行上位机机械臂命令。
 * @param 无。
 * @return 无。
 * @details 支持 P x y z、G x y z、OPEN、CLOSE。P/G 调用 Arm_MoveToXYZ()，G 成功后额外调用 Arm_GripClose()。
 * @note 本函数需要在主循环中周期调用，命令数据由 USART1_IRQHandler() 收集。
 */
void Arm_ProcessHostCommand(void)
{
    char *cursor;
    char command;
    float x;
    float y;
    float z;

    if (!s_host_cmd_ready)
    {
        return;
    }

    __disable_irq();
    strncpy(s_host_cmd_buf, (const char *)s_host_rx_buf, ARM_HOST_RX_BUF_SIZE);
    s_host_cmd_buf[ARM_HOST_RX_BUF_SIZE - 1U] = '\0';
    s_host_cmd_ready = 0;
    __enable_irq();

    cursor = s_host_cmd_buf;
    while (*cursor == ' ' || *cursor == '\t')
    {
        cursor++;
    }
    command = *cursor;

    if (command == 'P' || command == 'p' || command == 'G' || command == 'g')
    {
        cursor++;
        x = Arm_ParseNumber(&cursor);
        y = Arm_ParseNumber(&cursor);
        z = Arm_ParseNumber(&cursor);

        printf("Host target x=%.1f y=%.1f z=%.1f\r\n", x, y, z);
        if (Arm_MoveToXYZ(x, y, z) && (command == 'G' || command == 'g'))
        {
            Arm_GripClose();
        }
    }
    else if (strncmp(s_host_cmd_buf, "OPEN", 4) == 0 || strncmp(s_host_cmd_buf, "open", 4) == 0)
    {
        Arm_GripOpen();
    }
    else if (strncmp(s_host_cmd_buf, "CLOSE", 5) == 0 || strncmp(s_host_cmd_buf, "close", 5) == 0)
    {
        Arm_GripClose();
    }
    else
    {
        printf("Unknown cmd: %s\r\n", s_host_cmd_buf);
    }
}
