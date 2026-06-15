#include "Encoder.h"

void Encoder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |
						   RCC_APB2Periph_GPIOC |
						   RCC_APB2Periph_TIM8 |
						   RCC_APB2Periph_TIM10, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65535;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseInitStructure);
	TIM_EncoderInterfaceConfig(TIM10, TIM_EncoderMode_TI12,
							   TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_SetCounter(TIM10, 0);
	TIM_Cmd(TIM10, ENABLE);

	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);
	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12,
							   TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_SetCounter(TIM8, 0);
	TIM_Cmd(TIM8, ENABLE);
}

int16_t Encoder_Get(Encoder_Numtypedef n)
{
	int16_t Count = 0;

	if (n == Encoder_Left)
	{
		Count = (int16_t)TIM_GetCounter(TIM10);
		TIM_SetCounter(TIM10, 0);
	}
	else if (n == Encoder_Right)
	{
		Count = (int16_t)TIM_GetCounter(TIM8);
		TIM_SetCounter(TIM8, 0);
	}

	return Count;
}
