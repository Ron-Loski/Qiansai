#include "CallBack.h"
#include "Timer.h"

volatile uint8_t g_tim6_update_flag = 0;


/**
 * @brief TIM6 更新事件的软件回调函数。
 * @param 无。
 * @return 无。
 * @details 只将 g_tim6_update_flag 置 1，通知主循环可以读取 MPU6050 或执行周期任务。
 * @note 不在中断中读取 MPU6050，因为软件 I2C 内部会调用 Delay_Us()，放在中断里容易阻塞。
 */
void TIM6_UpdateCallBack(void)
{
    g_tim6_update_flag = 1;
}


/**
 * @brief TIM6 中断服务函数。
 * @param 无。
 * @return 无。
 * @details 判断 TIM_IT_Update 后清除挂起标志，再调用 TIM6_UpdateCallBack() 置位周期标志。
 */
void TIM6_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM6_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        TIM6_UpdateCallBack();
    }
}
