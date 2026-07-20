/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date                : 2021/06/06
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "debug.h"
#include "Global_Inc.h"

static const char *Navigation_ModeName(AStarMode mode)
{
    return (mode == ASTAR_MODE_FIXED_MAP) ? "FIXED_MAP" : "USART2_REALTIME";
}

static AStarMode Navigation_ToggleMode(AStarMode mode)
{
    return (mode == ASTAR_MODE_FIXED_MAP) ? ASTAR_MODE_USART2
                                          : ASTAR_MODE_FIXED_MAP;
}

int main(void)
{
#if 0
    uint32_t last_yaw_print = 0U;
#endif
    uint8_t navigation_running = 0U;
    AStarMode navigation_mode = ASTAR_MODE_FIXED_MAP;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);

    PWM_Init();
    Encoder_Init();
    Button_Init();
    BlueSerial_Init();
    BluetoothControl_Init();
    Arm_HostUartInit(115200);

    printf("Keep car still: calibrating MPU6050...\r\n");
    MPU6050_Init();
    AStar_ResetNavigation();
    Button_ClearEvents();
    /* Discard joystick frames that may have arrived during gyro calibration. */
    BluetoothControl_SetEnabled(1U);
    Arm_Init();

    printf("Navigation ready: PC2 active-low button\r\n");
    printf("Double click within 500ms: start/stop; single click: switch mode\r\n");
    printf("USART2 response: ASCII 00=free, 01=obstacle, optional CRLF\r\n");
    printf("USART3 Bluetooth: PC10=TX PC11=RX 9600, frame=[joystick,LH,LV,RH,RV]\r\n");
    printf("USART3 servo sliders (deg): [slider,1,J1] [slider,2,J2] "
           "[slider,3,J3] [slider,4,grip]\r\n");
    printf("Default mode=%s, start=(%d,%d), target=(%d,%d)\r\n",
           Navigation_ModeName(navigation_mode),
           robot_x, robot_y, target_x, target_y);

    while (1)
    {
        uint8_t button_events = Button_GetEvents();

        if (button_events & BUTTON_EVENT_DOUBLE)
        {
            PWM_Brake();
            if (navigation_running)
            {
                navigation_running = 0U;
                BluetoothControl_SetEnabled(1U);
                printf("Navigation stopped by double click at logical cell (%d,%d)\r\n",
                       robot_x, robot_y);
            }
            else
            {
                BluetoothControl_SetEnabled(0U);
                navigation_mode = ASTAR_MODE_FIXED_MAP;
                AStar_ResetNavigation();
                navigation_running = 1U;
                printf("Navigation started, mode=%s, start=(%d,%d), target=(%d,%d)\r\n",
                       Navigation_ModeName(navigation_mode),
                       robot_x, robot_y, target_x, target_y);
            }
            continue;
        }

        if ((button_events & BUTTON_EVENT_SINGLE) && navigation_running)
        {
            navigation_mode = Navigation_ToggleMode(navigation_mode);
            printf("Navigation mode switched to %s at cell (%d,%d)\r\n",
                   Navigation_ModeName(navigation_mode), robot_x, robot_y);
        }

        if (navigation_running)
        {
            AStarRunResult run_result = AStar_RunMode(navigation_mode);

            button_events = Button_GetEvents();
            if (button_events & BUTTON_EVENT_DOUBLE)
            {
                PWM_Brake();
                navigation_running = 0U;
                BluetoothControl_SetEnabled(1U);
                printf("Navigation stopped by double click at logical cell (%d,%d)\r\n",
                       robot_x, robot_y);
            }
            else if (button_events & BUTTON_EVENT_SINGLE)
            {
                navigation_mode = Navigation_ToggleMode(navigation_mode);
                printf("Navigation mode switched to %s at cell (%d,%d)\r\n",
                       Navigation_ModeName(navigation_mode), robot_x, robot_y);
            }
            else if (run_result == ASTAR_RUN_REACHED_TARGET)
            {
                PWM_Brake();
                navigation_running = 0U;
                BluetoothControl_SetEnabled(1U);
                printf("Navigation completed at target (%d,%d)\r\n", robot_x, robot_y);
            }
            else if (run_result == ASTAR_RUN_FAILED)
            {
                PWM_Brake();
                navigation_running = 0U;
                BluetoothControl_SetEnabled(1U);
                printf("Navigation failed in mode=%s at cell (%d,%d)\r\n",
                       Navigation_ModeName(navigation_mode), robot_x, robot_y);
            }
            else
            {
                PWM_Brake();
                navigation_running = 0U;
                BluetoothControl_SetEnabled(1U);
                printf("Navigation interrupted without a pending button event\r\n");
            }
        }
        else
        {
            BluetoothControl_Process();
            /* Do not let a USART1 yaw command overwrite active joystick PWM. */
            if (!BluetoothControl_IsDriving())
            {
                Arm_ProcessHostCommand();
            }
        }

        BluetoothControl_DebugStatus();

#if 0
        /* Temporarily disabled to keep USART1 output focused on Bluetooth debug. */
        if ((uint32_t)(MPU6050_YawSampleCount - last_yaw_print) >= 10U)
        {
            last_yaw_print = MPU6050_YawSampleCount;
            printf("Yaw=%.2f, GyroZ=%d, Tick=%lu\r\n",
                   AngleFilter.Yaw, MPU6050_Data.GyroZ,
                   (unsigned long)MPU6050_YawSampleCount);
        }
#endif
    }
}
