#include "PID.h"


/**
 * @brief 对 PID 中间量或输出做对称限幅。
 * @param value 待限幅的数值。
 * @param max 限幅绝对值；max<=0 时表示不启用限幅。
 * @return 限幅后的数值，范围为 -max~max。
 * @details 同时服务于积分限幅 ErrorInt 和最终输出限幅 Out，避免积分饱和或输出过大。
 */
static float PID_Limit(float value, float max)
{
    if (max <= 0.0f)
    {
        return value;
    }

    if (value > max)
    {
        return max;
    }
    if (value < -max)
    {
        return -max;
    }

    return value;
}


/**
 * @brief 初始化 PID 运行状态。
 * @param p PID_t 结构体指针，Kp/Ki/Kd 和限幅值可在调用前或调用后由用户赋值。
 * @return 无。
 * @details 清零目标值、反馈值、输出值、当前误差、上次误差和积分项，不修改 Kp/Ki/Kd/IntegralMax/OutputMax。
 */
void PID_Init(PID_t *p)
{
    if (p == 0)
    {
        return;
    }

    p->Target = 0.0f;
    p->Actual = 0.0f;
    p->Out = 0.0f;
    p->Error0 = 0.0f;
    p->Error1 = 0.0f;
    p->ErrorInt = 0.0f;
}


/**
 * @brief 执行一次位置式 PID 运算。
 * @param p PID_t 结构体指针；输入为 p->Target 和 p->Actual，输出写入 p->Out。
 * @return 无。
 * @details Error0=Target-Actual；Ki 不为 0 时累加 ErrorInt 并按 IntegralMax 积分限幅；
 *          Out=Kp*Error0+Ki*ErrorInt+Kd*(Error0-Error1)，最后按 OutputMax 输出限幅。
 * @note Motor.c 的 car_turn_to_roll() 使用本函数根据角度误差生成左右电机 PWM。
 */
void PID_Update(PID_t *p)
{
    if (p == 0)
    {
        return;
    }

    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    if (p->Ki != 0.0f)
    {
        p->ErrorInt += p->Error0;
        p->ErrorInt = PID_Limit(p->ErrorInt, p->IntegralMax);
    }
    else
    {
        p->ErrorInt = 0.0f;
    }

    p->Out = p->Kp * p->Error0
           + p->Ki * p->ErrorInt
           + p->Kd * (p->Error0 - p->Error1);

    p->Out = PID_Limit(p->Out, p->OutputMax);
}
