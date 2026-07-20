#include "ch32v30x.h"
#include "Blue.h"
#include "stdarg.h"
#include "Global_Inc.h"

char BlueSerial_RxPacket[100];
uint8_t BlueSerial_RxFlag;
volatile uint8_t isobstacle_flag = 0;
static volatile uint8_t s_obstacle_ascii_state = 0U;

void BlueSerial_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USART_InitStructure);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART2, ENABLE);
}

void BlueSerial_SendByte(uint8_t Byte)
{
	USART_SendData(USART2, Byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Array[i]);
	}
}

void BlueSerial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		BlueSerial_SendByte(String[i]);
	}
}

uint32_t BlueSerial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void BlueSerial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Number / BlueSerial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

void BlueSerial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	BlueSerial_SendString(String);
}

void BlueSerial_ResetObstacleResponse(void)
{
    __disable_irq();
    s_obstacle_ascii_state = 0U;
    isobstacle_flag = 0U;
    Rx_flag = 0U;
    __enable_irq();
}

static void BlueSerial_CompleteObstacleResponse(uint8_t obstacle)
{
    isobstacle_flag = (uint8_t)(obstacle != 0U);
    Rx_flag = 1U;
    s_obstacle_ascii_state = 0U;
}

void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(USART2);
        ch = data;

        /* Preferred response: ASCII "00"=free, "01"=obstacle. */
        if (data == 0x00U)
        {
            BlueSerial_CompleteObstacleResponse(0U);
        }
        else if (data == 0x01U)
        {
            BlueSerial_CompleteObstacleResponse(1U);
        }
        else if (s_obstacle_ascii_state == 0U)
        {
            if (data == (uint8_t)'0')
            {
                s_obstacle_ascii_state = 1U;
            }
            else if (data == (uint8_t)'1')
            {
                /* Compatibility with the former ASCII "1\r\n" response. */
                BlueSerial_CompleteObstacleResponse(1U);
            }
            else if (data != (uint8_t)'\r' && data != (uint8_t)'\n')
            {
                s_obstacle_ascii_state = 0U;
            }
        }
        else
        {
            if (data == (uint8_t)'0')
            {
                BlueSerial_CompleteObstacleResponse(0U);
            }
            else if (data == (uint8_t)'1')
            {
                BlueSerial_CompleteObstacleResponse(1U);
            }
            else if (data == (uint8_t)'\r' || data == (uint8_t)'\n')
            {
                /* Compatibility with the former ASCII "0\r\n" response. */
                BlueSerial_CompleteObstacleResponse(0U);
            }
            else
            {
                s_obstacle_ascii_state = 0U;
            }
        }

        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}