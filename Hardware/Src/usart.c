#include "ch32v30x.h"
#include "ch32v30x_usart.h"
#include "usart.h"
#include "Motor.h"
#include "Varibles.h"

uint8_t index_usar = 0;
uint8_t near[3];  uint8_t index_near = 0;
uint8_t mid[3];  uint8_t index_mid = 0;
uint8_t far[3];  uint8_t index_far = 0;
// uint8_t serial_flag = 0;
uint8_t isobstacle_flag;

/**
 * @brief 初始化 USART2 串口。
 * @param 无。
 * @return 无。
 * @details 配置 USART2 为 115200-8N1，并使能 RXNE 接收中断；引脚映射按原工程 USART2 重映射设置。
 * @note Astar/OpenMV 障碍物通信使用 USART2，机械臂上位机命令使用 USART1。
 */
void USART2_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

    // TX PD5
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // RX PD6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
}


/**
 * @brief 通过 USART2 发送字符串。
 * @param String 以 '\0' 结尾的字符串指针。
 * @return 无。
 * @details 逐字节调用 USART_SendData()，并等待 TXE 置位后发送下一个字节。
 */
void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        USART_SendData(USART2, String[i]);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    }
}


/**
 * @brief USART2 接收中断服务函数。
 * @param 无。
 * @return 无。
 * @details 收到 OpenMV 数据后读取一个字节到 ch，并置位 Rx_flag，供 Astar 等待逻辑判断数据已返回。
 */
void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        Rx_flag =1;
        ch = USART_ReceiveData(USART2);
        isobstacle_flag = ch;
        // USART_SendData(USART2, isobstacle_flag);
        // printf("isobstacle_flag:%d",isobstacle_flag);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

