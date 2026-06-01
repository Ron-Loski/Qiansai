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
int16_t AX, AY, AZ, GX, GY, GZ;			//定义用于存放各个数据的变量
uint8_t Data;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);	

	// MyI2C_Init();

	// MyI2C_Start();
	// MyI2C_SendByte(0xD0);
	// Ack = MyI2C_ReceiveAck();
	// MyI2C_Stop();

	MPU6050_Init();
	while(1)
    {
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);		//获取MPU6050的数据
		printf("%d,%d,%d,%d,%d,%d\r\n", AX, AY, AZ, GX, GY, GZ);

	}
}

