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
uint8_t ID;
uint8_t GPIO_State1 = 3;
uint8_t GPIO_State2 = 3;
uint8_t ID;								//定义用于存放ID号的变量
uint8_t Data;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);	

	MPU6050_Init();
	PWM_Init();

	PWM_EnableChannel(1, 800);
	PWM_EnableChannel(3, 500);




	while(1)
    {
		MPU6050_GetRoll();
		printf("%f\r\n", AngleFilter.Roll);
		// printf("%d,%d,%d,%d,%d,%d\r\n", MPU6050_Data.AccX, MPU6050_Data.AccY, MPU6050_Data.AccZ, MPU6050_Data.GyroX, MPU6050_Data.GyroY, MPU6050_Data.GyroZ);
	}
}

