#ifndef __BLUETOOTH_CONTROL_H_
#define __BLUETOOTH_CONTROL_H_

#include "ch32v30x.h"

#define BLUETOOTH_CONTROL_BAUDRATE  9600U

/*
 * USART3: PC10=TX, PC11=RX after partial remap.
 * Arm sliders use raw servo angles in degrees:
 * [slider,1,J1], [slider,2,J2], [slider,3,J3], [slider,4,gripper].
 */
void BluetoothControl_Init(void);
void BluetoothControl_SetEnabled(uint8_t enabled);
void BluetoothControl_Process(void);
void BluetoothControl_DebugStatus(void);
void BluetoothControl_Stop(void);
uint8_t BluetoothControl_IsDriving(void);

void BluetoothControl_SendByte(uint8_t data);
void BluetoothControl_SendString(const char *string);
void BluetoothControl_Printf(const char *format, ...);

#endif
