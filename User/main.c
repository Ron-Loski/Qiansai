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
#include "Astar.h"
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
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);	
	// printf("SystemClk:%d\r\n",SystemCoreClock);
	// printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
	// printf("This is printf example\r\n");
    PWM_Init();
    Encoder_Init();
    USART2_Init();
    MPU6050_Init();
    TIM6_Init();
    Arm_HostUartInit(115200);
    // Arm_Init();
while(1)
{
        if (g_tim6_update_flag)
        {
            g_tim6_update_flag = 0;
            MPU6050_GetRoll();
            printf("%f\r\n", AngleFilter.Roll);
        }
        // Arm_ProcessHostCommand();    //?????????
        // ArmServo_SetAngle(4U, 50);
}

}
/*   ?????????   */
//     int8_t target_x = 5;
//     int8_t target_y = 5;

//     int8_t robot_x0 = 0;
//     int8_t robot_y0 = 0;

// 	g_opencount = 0;
//     g_closedcount = 0;

//     // if (isobstacle[robot_x0][robot_y0] == 1 || isobstacle[target_x][target_y] == 1)
//     // {
//     //     return 0;
//     // }

//     float h0 = distance(robot_x0, robot_y0,target_x, target_y);
//     insertopen(robot_x0, robot_y0, robot_x0, robot_y0, 0.0f, h0, h0);

//     g_closedlist[g_closedcount].x = robot_x0;
//     g_closedlist[g_closedcount].y = robot_y0;

//     g_closedcount++;
//     g_openlist[0].on_list = 1;                                                   //?????openm??
    
//     int8_t robot_x = robot_x0;
//     int8_t robot_y = robot_y0;
// 	uint16_t itr_count = 0;

// // printf("target_x:%d,target_y:%d", target_x,target_y);
// 	while(robot_x != target_x || robot_y != target_y || itr_count >=5000)
//     {
// 	    itr_count ++;
//         uint16_t min_index = findminfnode();
//         if (min_index == 0xFFFF)
//         {
//             printf("jump\r\n");
//             break;                      //??????????
//         }
//         opennode*curnode = &g_openlist[min_index];   //????f????????
//         tempminf_x = curnode->x;                            
//         tempminf_y = curnode->y;

//         g_closedlist[g_closedcount].x = tempminf_x;      //??????closed??
//         g_closedlist[g_closedcount].y = tempminf_y;
//         g_closedcount ++;
//         g_openlist[min_index].on_list = 0;
// /*    ?????????????f????    */
//         car_move(robot_x, robot_y, tempminf_x, tempminf_y);                                                        
//         robot_x = tempminf_x;
//         robot_y = tempminf_y;

// /*    ?????????????????????????????    */
//         car_return(g_openlist[min_index].parent_x, g_openlist[min_index].parent_y, robot_x, robot_y);

//         expandnode(curnode, robot_x0, robot_y0, target_x, target_y);
//     }

// 	}





