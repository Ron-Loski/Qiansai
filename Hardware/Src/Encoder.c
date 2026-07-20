#include "Encoder.h"
#include "Motor.h"
#define ENCODER_TIM_PERIOD    0xFFFF
#define ENCODER_IC_FILTER     10


void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap_TIM8, DISABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM10, DISABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_Period = ENCODER_TIM_PERIOD;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);
    TIM_TimeBaseInit(TIM10, &TIM_TimeBaseInitStructure);

    TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM10, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = ENCODER_IC_FILTER;

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(TIM8, &TIM_ICInitStructure);
    TIM_ICInit(TIM10, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM8, &TIM_ICInitStructure);
    TIM_ICInit(TIM10, &TIM_ICInitStructure);

    TIM_SetCounter(TIM8, 0);
    TIM_SetCounter(TIM10, 0);

    TIM_Cmd(TIM8, ENABLE);
    TIM_Cmd(TIM10, ENABLE);
}

int16_t Encoder_GetCount(uint8_t Encoder)
{
    if(Encoder == ENCODER_1)
    {
        return (int16_t)TIM_GetCounter(TIM8);
    }
    else if(Encoder == ENCODER_2)
    {
        return (int16_t)TIM_GetCounter(TIM10);
    }

    return 0;
}

