#include "Motor.h"
#include "Astar.h"
#include "PID.h"
#include "MPU6050.h"
#include "CallBack.h"

/**
 * @brief 初始化小车电机 PWM 硬件。
 * @param 无。
 * @return 无。
 * @details 打开 GPIOA、AFIO、TIM2 时钟，将 TIM2_CH1~CH4 配置为 PWM1 模式。
 *          当前 TIM2 计数频率为 100kHz，周期为 100 个计数，所以 Compare 通常按 0~100 理解为占空比。
 *          PA0~PA3 同时承担 H 桥方向脚和 PWM 输出脚，实际输出通道由 PWM_EnableChannel() 动态切换。
 * @note 使用 forward_dis()、turnround_right()、turnround_left()、car_turn_to_roll() 前必须先调用本函数。
 */
void PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_3);

    TIM_TimeBaseInitStructure.TIM_Prescaler = SystemCoreClock / 100000 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief 关闭/制动两个电机输出。
 * @param 无。
 * @return 无。
 * @details 将 PA0~PA3 重新配置为普通推挽输出并全部置高，沿用原工程的电机制动方式。
 * @note PID 转向达到角度阈值后会调用本函数，满足“到位后关掉两个电机输出”的要求。
 */
void PWM_Brake(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
}

/**
 * @brief 使能指定电机、指定方向的 PWM 输出。
 * @param Motor_Num 电机编号，Motor_Left 表示左电机，Motor_Right 表示右电机。
 * @param Compare PWM 占空比比较值，当前 TIM2 周期为 100，因此通常取 0~100。
 * @param Direct 电机方向，Forward 为正转，Reverse 为反转。
 * @return 无。
 * @details 根据电机和方向选择 TIM2_CH1~CH4 中的一个通道输出 PWM，同时把另一侧方向脚置为普通输出高电平。
 * @note forward_dis()、car_turn_drive() 都通过本函数驱动电机，不直接操作 TIM2 比较寄存器。
 */
void PWM_EnableChannel(Motor_Numtypedef Motor_Num, uint16_t Compare, Motor_Directtypedef Direct)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    if (Direct == Forward)
    {
        if (Motor_Num == Motor_Left)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);

            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            TIM_SetCompare2(TIM2, Compare);
            TIM_CCxCmd(TIM2, TIM_Channel_2, TIM_CCx_Enable);
        }
        if (Motor_Num == Motor_Right)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_SET);

            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            TIM_SetCompare3(TIM2, Compare);
            TIM_CCxCmd(TIM2, TIM_Channel_3, TIM_CCx_Enable);
        }
    }
    if (Direct == Reverse)
    {
        if (Motor_Num == Motor_Left)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);

            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            TIM_SetCompare1(TIM2, Compare);
            TIM_CCxCmd(TIM2, TIM_Channel_1, TIM_CCx_Enable);
        }
        if (Motor_Num == Motor_Right)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);

            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            TIM_SetCompare4(TIM2, Compare);
            TIM_CCxCmd(TIM2, TIM_Channel_4, TIM_CCx_Enable);
        }
    }

    TIM_Cmd(TIM2, ENABLE);
}

volatile uint8_t stop_flag = 0;
static PID_t s_car_turn_pid;
static uint8_t s_car_turn_pid_ready = 0;

/**
 * @brief 求浮点数绝对值。
 * @param value 输入值。
 * @return value 的绝对值。
 * @details 用于把 PID 输出的绝对值转换为 PWM 大小，正负号本身用于决定左右转方向。
 */
static float car_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 将角度归一化到 -180~180 度。
 * @param angle 原始角度，单位：度。
 * @return 归一化后的角度，单位：度。
 * @details 解决目标角和当前角跨越 180/-180 边界时的最短方向误差计算问题。
 */
static float car_normalize_angle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief 将 Astar 原有的 11/22/33/44 转向编码转换为目标角度。
 * @param turn_code 转向编码，11/22/33/44 分别表示 45/90/135/180 度。
 * @return 对应转角，单位：度。
 * @details 保持原 Astar 调用习惯不变：turnround_right(22.0) 仍表示转 90 度。
 */
static float car_turn_code_to_angle(uint8_t turn_code)
{
    switch (turn_code)
    {
    case 11:
        return 45.0f;
    case 22:
        return 90.0f;
    case 33:
        return 135.0f;
    case 44:
        return 180.0f;
    default:
        return ((float)turn_code) * 45.0f / 11.0f;
    }
}

/**
 * @brief 初始化小车转向 PID 参数和状态。
 * @param 无。
 * @return 无。
 * @details 清空 PID 历史误差和积分项，再写入 Motor.h 中定义的 Kp/Ki/Kd、积分限幅和输出限幅。
 * @note 每次按 11/22/33/44 开始一次新的闭环转向前调用，避免上一次转向的积分和误差影响本次动作。
 */
static void car_turn_pid_prepare(void)
{
    PID_Init(&s_car_turn_pid);
    s_car_turn_pid.Kp = CAR_TURN_PID_KP;
    s_car_turn_pid.Ki = CAR_TURN_PID_KI;
    s_car_turn_pid.Kd = CAR_TURN_PID_KD;
    s_car_turn_pid.IntegralMax = CAR_TURN_PID_INTEGRAL_MAX;
    s_car_turn_pid.OutputMax = CAR_TURN_PID_OUTPUT_MAX;
    s_car_turn_pid_ready = 1;
}

/**
 * @brief 根据 PID 输出驱动左右电机原地转向。
 * @param pid_out PID 输出值，正负号表示转向方向，绝对值表示 PWM 强度。
 * @return 无。
 * @details 输出值会被限制在 CAR_TURN_PWM_MIN~CAR_TURN_PWM_MAX 之间，避免 PWM 太小电机不动或太大导致过冲。
 * @note 如果实车发现转向方向和目标相反，就交换本函数中 pid_out >= 0 分支与 else 分支的左右电机方向。
 */
static void car_turn_drive(float pid_out)
{
    uint16_t pwm = (uint16_t)car_absf(pid_out);

    if (pwm < CAR_TURN_PWM_MIN)
    {
        pwm = CAR_TURN_PWM_MIN;
    }
    if (pwm > CAR_TURN_PWM_MAX)
    {
        pwm = CAR_TURN_PWM_MAX;
    }

    if (pid_out >= 0.0f)
    {
        PWM_EnableChannel(Motor_Left, pwm, Forward);
        PWM_EnableChannel(Motor_Right, pwm, Reverse);
    }
    else
    {
        PWM_EnableChannel(Motor_Left, pwm, Reverse);
        PWM_EnableChannel(Motor_Right, pwm, Forward);
    }
}

/**
 * @brief 让小车朝向闭环逼近指定 Roll 目标角。
 * @param target_roll 目标朝向角，单位：度，通常由 CAR_DEFAULT_ROLL_DEG 加减 45/90/135/180 得到。
 * @return Motor_TurnBusy 表示还未到位；Motor_TurnOK 表示已进入角度阈值范围并已调用 PWM_Brake() 停车。
 * @details 利用 TIM6 周期标志 g_tim6_update_flag 控制采样节奏；每次标志有效时读取 MPU6050 更新 AngleFilter.Roll，
 *          再计算 target_roll - AngleFilter.Roll 的角度误差，送入 PID_Update() 得到左右电机 PWM。
 * @note 角度误差小于 CAR_TURN_ANGLE_THRESHOLD_DEG 即认为到位，不要求机械小车完全静止在单一角度点。
 */
Motor_TurnStatustypedef car_turn_to_roll(float target_roll)
{
    float error;

    if (!s_car_turn_pid_ready)
    {
        car_turn_pid_prepare();
    }

    if (g_tim6_update_flag == 0)
    {
        return Motor_TurnBusy;
    }

    g_tim6_update_flag = 0;
    MPU6050_GetRoll();

    error = car_normalize_angle(target_roll - AngleFilter.Roll);
    if (car_absf(error) <= CAR_TURN_ANGLE_THRESHOLD_DEG)
    {
        PWM_Brake();
        return Motor_TurnOK;
    }

    s_car_turn_pid.Target = error;
    s_car_turn_pid.Actual = 0.0f;
    PID_Update(&s_car_turn_pid);
    car_turn_drive(s_car_turn_pid.Out);

    return Motor_TurnBusy;
}

/**
 * @brief 按 Astar 原有 11/22/33/44 编码执行一次阻塞式闭环转向。
 * @param turn_code 转向编码：11=45度，22=90度，33=135度，44=180度。
 * @param direct 转向方向：Forward 按右转目标计算，Reverse 按左转目标计算。
 * @return Motor_TurnOK，函数内部会阻塞直到转向到位。
 * @details 先把编码转换为相对默认朝向 CAR_DEFAULT_ROLL_DEG 的目标角，再循环调用 car_turn_to_roll()。
 * @note turnround_right() 和 turnround_left() 调用本函数，因此 Astar 原代码不用改调用形式。
 */
Motor_TurnStatustypedef car_turn_by_code(uint8_t turn_code, Motor_Directtypedef direct)
{
    float angle = car_turn_code_to_angle(turn_code);
    float target_roll;

    car_turn_pid_prepare();

    if (direct == Reverse)
    {
        angle = -angle;
    }

    target_roll = car_normalize_angle(CAR_DEFAULT_ROLL_DEG + angle);
    while (car_turn_to_roll(target_roll) != Motor_TurnOK)
    {
    }

    return Motor_TurnOK;
}

/**
 * @brief 控制小车直行指定距离。
 * @param distance 目标直行距离，单位沿用原工程计算公式，结合 girth 换算为编码器计数。
 * @return 无。
 * @details 左右电机同时正转，通过 TIM8/TIM10 编码器计数判断是否到达目标距离，到达后调用 PWM_Brake() 停车。
 * @note car_move() 在完成转向后调用本函数走到相邻格子。
 */
void forward_dis(float distance)
{
    TIM8->CNT = 0;
    TIM10->CNT = 0;
    stop_flag = 0;
    uint16_t target_cnt = (uint16_t)distance * 12.51 * 4 * 30 / girth;

    while (stop_flag == 0)
    {
        PWM_EnableChannel(Motor_Left, 50, Forward);
        PWM_EnableChannel(Motor_Right, 50, Forward);
        if (TIM8->CNT >= target_cnt && TIM10->CNT >= target_cnt)
        {
            stop_flag = 1;
            TIM8->CNT = 0;
            TIM10->CNT = 0;
        }
    }

    stop_flag = 0;
    PWM_Brake();
}

/**
 * @brief 右转指定编码角度。
 * @param distance 原工程的转向编码，11/22/33/44 分别对应右转 45/90/135/180 度。
 * @return 无。
 * @details 当前已改为调用 car_turn_by_code() 做 Roll 角度 PID 闭环，到位后自动 PWM_Brake()。
 */
void turnround_right(float distance)
{
    car_turn_by_code((uint8_t)(distance + 0.5f), Forward);
}

/**
 * @brief 左转指定编码角度。
 * @param distance 原工程的转向编码，11/22/33/44 分别对应左转 45/90/135/180 度。
 * @return 无。
 * @details 当前已改为调用 car_turn_by_code() 做 Roll 角度 PID 闭环，到位后自动 PWM_Brake()。
 */
void turnround_left(float distance)
{
    car_turn_by_code((uint8_t)(distance + 0.5f), Reverse);
}

/**
 * @brief 根据当前格子和目标相邻格子执行一次移动。
 * @param cur_x 当前 x 坐标。
 * @param cur_y 当前 y 坐标。
 * @param minf_x 下一目标格子 x 坐标。
 * @param minf_y 下一目标格子 y 坐标。
 * @return 无。
 * @details 根据 dx/dy 判断八方向移动，先调用 turnround_left/right() 调整朝向，再调用 forward_dis() 前进一格或斜向距离。
 * @note 由 AStar_plan() 在选出 f 值最小节点后调用。
 */
void car_move(uint16_t cur_x, uint16_t cur_y, uint16_t minf_x, uint16_t minf_y)
{
    int8_t dx = (int8_t)(minf_x - cur_x);
    int8_t dy = (int8_t)(minf_y - cur_y);

    if (dx == -1)
    {
        if (dy == 1)
        {
            printf("leftupmove\r\n");
            turnround_left(11.0);
            forward_dis(steplen * 1.414);
        }
        else if (dy == 0)
        {
            printf("leftmove\r\n");
            turnround_left(22.0);
            forward_dis(steplen);
        }
        else
        {
            printf("leftdownmove\r\n");
            turnround_left(33.0);
            forward_dis(steplen * 1.414);
        }
    }
    else if (dx == 0)
    {
        if (dy == 1)
        {
            printf("upmove\r\n");
            forward_dis(steplen);
        }
        else if (dy == -1)
        {
            printf("downmove\r\n");
            turnround_left(44.0);
            forward_dis(steplen);
        }
        else
        {
            return;
        }
    }
    else
    {
        if (dy == 1)
        {
            printf("rightupmove\r\n");
            turnround_right(11.0);
            forward_dis(steplen * 1.414);
        }
        else if (dy == 0)
        {
            printf("rightmove\r\n");
            turnround_right(22.0);
            forward_dis(steplen);
        }
        else
        {
            printf("rightdownmove\r\n");
            turnround_right(33.0);
            forward_dis(steplen * 1.414);
        }
    }
}

/**
 * @brief Astar 扩展节点时把车头转向指定八方向。
 * @param a 方向索引：0上，1右上，2右，3右下，4下，5左下，6左，7左上。
 * @return 无。
 * @details 0 不转；1~4 调用右转 45/90/135/180 度；5~7 调用左转 135/90/45 度。
 *          转向完成后通过 USART2 向 OpenMV 发送 0x00 或 0xFF，区分水平/垂直格与斜向格。
 * @note expandnode() 调用本函数，使摄像头或传感器朝向待检测的新节点方向。
 */
void car_turn(uint8_t a)
{
    if (a == 0)
    {
        printf("direction 0: forward, no turn\r\n");
    }
    else if (a <= 4)
    {
        turnround_right((float)(a * 11U));
    }
    else
    {
        turnround_left((float)((8U - a) * 11U));
    }

    if (a % 2 == 0)
    {
        USART_SendData(USART2, 0x00);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    }
    else
    {
        USART_SendData(USART2, 0xFF);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    }
}

/**
 * @brief 小车移动后根据父节点方向回正到默认朝向。
 * @param par_x 父节点 x 坐标。
 * @param par_y 父节点 y 坐标。
 * @param x0 当前节点 x 坐标。
 * @param y0 当前节点 y 坐标。
 * @return 无。
 * @details 根据当前节点相对父节点的 dx/dy 判断刚才走来的方向，再调用 turnround_right/left() 转回默认前方。
 * @note AStar_plan() 在 car_move() 后调用本函数，保证下一轮 expandnode() 从统一默认朝向开始扫描。
 */
void car_return(int8_t par_x, int8_t par_y, int8_t x0, int8_t y0)
{
    int8_t dx = x0 - par_x;
    int8_t dy = y0 - par_y;

    if (dx == -1)
    {
        if (dy == 1)
        {
            printf("turnright1hui\r\n");
            turnround_right(11.0);
        }
        else if (dy == 0)
        {
            printf("turnright2hui");
            turnround_right(22.0);
        }
        else
        {
            printf("turnright3hui");
            turnround_right(33.0);
        }
    }
    else if (dx == 0)
    {
        if (dy == -1)
        {
            printf("turnright4hui");
            turnround_right(44.0);
        }
        else if (dy == 1)
        {
            return;
        }
    }
    else
    {
        if (dy == 1)
        {
            printf("turnleft1hui\r\n");
            turnround_left(11.0);
        }
        else if (dy == 0)
        {
            printf("turnleft2hui\r\n");
            turnround_left(22.0);
        }
        else
        {
            printf("turnleft3hui\r\n");
            turnround_left(33.0);
        }
    }
}
