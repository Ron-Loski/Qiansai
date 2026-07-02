#include "Timer.h"

#define TIM6_TICK_HZ        10000U
#define TIM6_PERIOD_TICKS   100U


/**
 * @brief 初始化 TIM6 为 10ms 周期更新中断。
 * @param 无。
 * @return 无。
 * @details 将 TIM6 计数频率配置为 10kHz，周期 100 个计数，即 10ms 触发一次更新中断，并打开 NVIC。
 * @note TIM6_IRQHandler() 在 CallBack.c 中实现，中断内只置位 g_tim6_update_flag，耗时任务放在主循环执行。
 */
void TIM6_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_Prescaler = (uint16_t)(SystemCoreClock / TIM6_TICK_HZ - 1U);
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = TIM6_PERIOD_TICKS - 1U;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);

    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM6, ENABLE);
}


/**
 * @brief 启动 TIM6 计数。
 * @param 无。
 * @return 无。
 * @details 清零计数器和更新中断挂起标志后使能 TIM6。
 */
void TIM6_Start(void)
{
    TIM_SetCounter(TIM6, 0);
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    TIM_Cmd(TIM6, ENABLE);
}


/**
 * @brief 停止 TIM6 计数。
 * @param 无。
 * @return 无。
 * @details 关闭 TIM6 并清除更新中断挂起标志，避免下次启动立即进入中断。
 */
void TIM6_Stop(void)
{
    TIM_Cmd(TIM6, DISABLE);
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
}
