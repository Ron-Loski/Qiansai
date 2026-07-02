/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 USART Print debugging routine:
 USART1_Tx(PA9).
 This example demonstrates using USART1(PA9) as a print debug port output.

*/

#include "debug.h"
#include "Global_Inc.h"
#include "Varibles.h"

/* Global typedef */

/* Global define */

/* Global Variable */


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */

uint8_t Ack = 3;
uint8_t GPIO_State1 = 3;
uint8_t GPIO_State2 = 3;
uint8_t ID;								//定义用于存放ID号的变量
uint8_t Data;
uint16_t Count1 = 0;
uint16_t Count2 = 0;
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);	

	MPU6050_Init();
	PWM_Init();

	// Timer_Init();


	while(1)
    {
		MPU6050_GetRoll();
		// printf("%d,%d,%d,%d,%d,%d\r\n", MPU6050_Data.AccX, MPU6050_Data.AccY, MPU6050_Data.AccZ, MPU6050_Data.GyroX, MPU6050_Data.GyroY, MPU6050_Data.GyroZ);
		printf("%f\r\n", AngleFilter.Roll);
	}
}


void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*TIM3中断服务函数,用于PID调控*/
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		Count1 ++;
		Count2 ++;
		if (Count1 > 10)
		{
			Count1 = 0;
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

			MPU6050_GetRoll();

			AngleLoop.Actual = AngleFilter.Roll;
			PID_Update(&AngleLoop);
			AvePWM = AngleLoop.Out;

			LeftPWM = AvePWM + DifPWM / 2.0;
			RightPWM = AvePWM - DifPWM / 2.0;
			if (LeftPWM >= 0){
				PWM_EnableChannel(Motor_Left, LeftPWM, Forward);
			}else{
				PWM_EnableChannel(Motor_Left, LeftPWM, Reverse);
			}

			if (RightPWM >= 0){
				PWM_EnableChannel(Motor_Right, RightPWM, Forward);
			}else{
				PWM_EnableChannel(Motor_Right, RightPWM, Reverse);
			}

		}
		
		if (Count2 > 50)
		{
			Count2 = 0;

			SpeedLeft = Encoder_Get(1) / 44.0 / 9.27666;
			SpeedLeft = Encoder_Get(1) / 44.0 / 9.27666;
			
			AveSpeed = (SpeedLeft + SpeedRight) / 2.0;
			DifSpeed = SpeedLeft - SpeedRight;
			
			
			SpeedLoop.Actual = AveSpeed;
			PID_Update(&SpeedLoop);
			SpeedLoop.Target = SpeedLoop.Out;
			
			TurnLoop.Actual = DifSpeed;
			PID_Update(&TurnLoop);
			DifPWM = TurnLoop.Out;
		}

		


		/* Add PID control code here. */
	}
}
